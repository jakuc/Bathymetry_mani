#!/usr/bin/env python3
"""
publish_cloud_csv.py - publikuje ZAPISANĄ chmurę z CSV kolektora jako PointCloud2.

Po co: RViz pokazuje ostatnią /laser_cloud, jaka DOTARŁA. Cała chmura to jedna
duża wiadomość (~47 kB przy 3000 punktów), a przez WiFi DDS dzieli ją na
fragmenty - zgubiony fragment odrzuca całą wiadomość i RViz zostaje przy
starszej, niepełnej wersji. Po zatrzymaniu stacku nic jej już nie odświeży.
Ten skrypt pokazuje to, co NAPRAWDĘ jest w pliku, publikując lokalnie.

Pole `intensity` = signal_quality (u JRT mniej = lepiej), żeby w RViz dało się
pokolorować punkty po jakości.

  python3 publish_cloud_csv.py plik.csv [--topic /saved_cloud] [--frame base_link]
"""

import argparse
import csv
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def load(path, color, max_range, only_kept):
    pts = []
    dropped_keep = dropped_range = 0
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                x, y, z, d = float(row["x"]), float(row["y"]), float(row["z"]), float(row["d"])
            except (KeyError, ValueError):
                continue
            # Kolumna `keep` z cloud_isolation.py --write (kryterium łączone).
            if only_kept and row.get("keep", "1") == "0":
                dropped_keep += 1
                continue
            # TYLKO DO PODGLĄDU: odcięcie po odległości, żeby kilka powtarzalnych
            # odczytów ~35-40 m nie zjadało skali kolorów. Pliki i filtry
            # analityczne kryterium odległości NIE używają (decyzja usera).
            if max_range > 0 and d > max_range:
                dropped_range += 1
                continue
            if color == "distance":
                val = d
            else:
                s = row.get("signal_quality", "")
                val = float(s) if s not in ("", "None") else math.nan
            pts.append((x, y, z, val))
    return pts, dropped_keep, dropped_range


def to_msg(pts, frame, stamp):
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(pts)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("ffff", *p) for p in pts)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+", help="plik[:topic] - bez topicu idzie na --topic")
    ap.add_argument("--topic", default="/saved_cloud")
    ap.add_argument("--frame", default="base_link")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--color", choices=["sq", "distance"], default="sq",
                    help="co trafia do pola intensity: jakość sygnału albo odległość")
    ap.add_argument("--max-range", type=float, default=0.0,
                    help="TYLKO PODGLĄD: pomiń punkty dalsze niż tyle metrów (0 = bez limitu)")
    ap.add_argument("--only-kept", action="store_true",
                    help="pomiń punkty z keep=0 (plik z cloud_isolation.py --write)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("publish_cloud_csv")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    items = []
    for spec in args.csv:
        path, _, topic = spec.partition(":")
        pts, dk, dr = load(path, args.color, args.max_range, args.only_kept)
        items.append((node.create_publisher(PointCloud2, topic or args.topic, qos), pts, topic or args.topic))
        node.get_logger().info(
            f"{path}: {len(pts)} punktów -> {topic or args.topic} (pominięte: filtr {dk}, "
            f"ponad {args.max_range:g} m tylko w podglądzie {dr}; kolor: {args.color})")

    def tick():
        stamp = node.get_clock().now().to_msg()
        for pub, pts, _ in items:
            pub.publish(to_msg(pts, args.frame, stamp))

    tick()
    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
