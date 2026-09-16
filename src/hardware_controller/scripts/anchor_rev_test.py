#!/usr/bin/env python3
"""
anchor_rev_test - TEST REWERSYJNY: wyznacza kotwicę czasu pomiaru i luz osi.

PO CO. Punkt chmury powstaje z pary (odległość, kąt). Kąt bierze się z TF po
stemplu czasu pomiaru, więc błąd czasu JEST błędem kąta - przy 24 st/s każde
10 ms to 0,24 stopnia. Stempel z Nano mówi, kiedy przyszedł pierwszy bajt
linii, a nie kiedy moduł zbierał światło. Różnica jest stała i właśnie ją
mierzy ten test.

METODA. Ten sam wiersz elewacji przejeżdżamy W OBIE STRONY ze stałą prędkością.
Błąd czasu `dt` przesuwa punkty o `omega*dt` W KIERUNKU JAZDY, więc ta sama
ściana wychodzi w dwóch miejscach oddalonych o `Delta = 2*omega*dt`. Błąd
GEOMETRYCZNY (przesunięcie montażu, złe zero, skala kąta) nie zmienia znaku
z kierunkiem, więc sam się znosi - dlatego test mierzy SPÓJNOŚĆ, nie dokładność.

KILKA PRĘDKOŚCI, NIE JEDNA. Przy jednej prędkości nie da się odróżnić błędu
czasu od luzu mechanicznego - oba zależą od kierunku. Rozdziela je dopiero
model `Delta(omega) = 2*dt*omega + b`: nachylenie to czas, wyraz wolny to luz.
Stąd domyślne trzy prędkości.

Test zapisuje SUROWE dane (czas pomiaru, odległość, kąty obu osi, kierunek,
prędkość zadana) do CSV; liczeniem zajmuje się scripts/anchor_from_rev.py.
Rozdzielenie jest celowe: analizę wolno powtarzać i poprawiać bez ruszania
sprzętem.

Kąt pod pomiar liczymy SAMI, interpolując /joint_states na stempel pomiaru -
tak samo, jak zrobiłby to TF, ale bez zależności od drzewa TF. Dzięki temu
wynik testu nie zależy od tego, czy laser_xyz jest już zmierzone.

Uruchomienie (stack musi już działać, z use_laser i obiema osiami):
  ros2 run hardware_controller anchor_rev_test --ros-args \
      -p el_min_deg:=-40.0 -p el_max_deg:=40.0 -p speeds:="[6.0,12.0,24.0]"
"""

import csv
import math
import os
import threading
from datetime import datetime

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64MultiArray


class AnchorRevTest(Node):
    def __init__(self):
        super().__init__("anchor_rev_test")

        self.declare_parameter("azimuth_joint", "xm540_joint_z")
        self.declare_parameter("elevation_joint", "xm540_joint")
        self.declare_parameter("joint_order", ["xm540_joint", "xm540_joint_z"])
        self.declare_parameter("az_deg", 0.0)
        self.declare_parameter("el_min_deg", -40.0)
        self.declare_parameter("el_max_deg", 40.0)
        # Trzy prędkości, bo dwie wyznaczają prostą bez żadnej nadmiarowości,
        # a trzecia pozwala sprawdzić, czy model liniowy w ogóle się trzyma.
        self.declare_parameter("speeds", [6.0, 12.0, 24.0])
        # Więcej przejazdów przy większych prędkościach: gęstość próbkowania
        # spada wprost proporcjonalnie do prędkości, a to ONA (nie sam efekt)
        # psuła wcześniejsze podejście - rzadka siatka sama dodaje rozrzut.
        self.declare_parameter("passes_per_direction", 2)
        self.declare_parameter("equalize_density", True)
        self.declare_parameter("travel_speed_deg_s", 40.0)
        self.declare_parameter("accel_deg_s2", 60.0)
        self.declare_parameter("cmd_rate", 50.0)
        self.declare_parameter("settle_time", 0.6)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("file_prefix", "anchor_rev")

        self._az_joint = self.get_parameter("azimuth_joint").value
        self._el_joint = self.get_parameter("elevation_joint").value
        self._order = list(self.get_parameter("joint_order").value)
        self._az = self.get_parameter("az_deg").value
        self._el_min = self.get_parameter("el_min_deg").value
        self._el_max = self.get_parameter("el_max_deg").value
        self._speeds = [float(s) for s in self.get_parameter("speeds").value]
        self._passes = int(self.get_parameter("passes_per_direction").value)
        self._equalize = self.get_parameter("equalize_density").value
        self._travel = self.get_parameter("travel_speed_deg_s").value
        self._accel = self.get_parameter("accel_deg_s2").value
        self._cmd_dt = 1.0 / self.get_parameter("cmd_rate").value
        self._settle = self.get_parameter("settle_time").value
        self._prefix = self.get_parameter("file_prefix").value
        self._out_dir = self.get_parameter("output_dir").value or os.environ.get(
            "ROS_LOG_DIR", os.path.expanduser("~/scans"))

        self._positions = {}
        # Historia pozycji osi: (czas, elewacja, azymut). Z niej interpolujemy
        # kąt na stempel pomiaru.
        self._hist = []
        self._hist_max = 20000
        self._raw = {}
        self._rows = []
        self._lock = threading.Lock()
        self._collect = False
        self._meta = (0.0, 0)          # (prędkość zadana, numer przejazdu)
        self._stop = threading.Event()
        self.done = threading.Event()

        self._pub_cmd = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)
        self._pub_pvel = self.create_publisher(
            Float64MultiArray, "/profile_velocity_controller/commands", 10)
        self._pub_pacc = self.create_publisher(
            Float64MultiArray, "/profile_acceleration_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, 50)
        self.create_subscription(Range, "/laser/range", self._cb_range, 50)
        self.create_subscription(Vector3Stamped, "/laser/raw", self._cb_raw, 50)

        threading.Thread(target=self._run, daemon=True).start()

    # ------------------------------------------------------------------ wejścia

    def _cb_joints(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = pos
        el = self._positions.get(self._el_joint)
        az = self._positions.get(self._az_joint)
        if el is None or az is None:
            return
        with self._lock:
            self._hist.append((t, math.degrees(el), math.degrees(az)))
            if len(self._hist) > self._hist_max:
                del self._hist[:len(self._hist) - self._hist_max]

    def _cb_raw(self, msg: Vector3Stamped):
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._raw[key] = (msg.vector.x, msg.vector.y, msg.vector.z)
        if len(self._raw) > 4096:
            self._raw.pop(next(iter(self._raw)))

    def _interp(self, t):
        """Kąty obu osi w chwili t, z interpolacji /joint_states. None = brak
        próbek po obu stronach, czyli pomiar spoza okna przejazdu."""
        with self._lock:
            hist = list(self._hist)
        if len(hist) < 2 or t < hist[0][0] or t > hist[-1][0]:
            return None
        lo, hi = 0, len(hist) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if hist[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, el0, az0 = hist[lo]
        t1, el1, az1 = hist[hi]
        if t1 - t0 <= 0.0:
            return el0, az0, 0.0
        f = (t - t0) / (t1 - t0)
        el_t = el0 + f * (el1 - el0)
        # Prędkość osi w tej chwili, z okna 200 ms WSTECZ - nie z dwóch sąsiednich
        # próbek. Jeden tick enkodera (0,088 st) w 20 ms to skok 4,4 st/s, więc
        # prędkość z sąsiednich próbek przyjmowała tylko 0 / 4,4 / 8,8 ... st/s
        # i filtr jazdy ze stałą prędkością zostawiał wyłącznie wyższy próg
        # (zmierzone 2026-09-17: 5,5 st/s raportowane jako 8,77). W 200 ms kwant
        # to 0,44 st/s. Okno wstecz, bo próbek z przyszłości jeszcze nie ma.
        t_back = t - 0.2
        k = lo
        while k > 0 and hist[k][0] > t_back:
            k -= 1
        if hist[k][0] <= t_back and k + 1 < len(hist) and hist[k + 1][0] > hist[k][0]:
            fb = (t_back - hist[k][0]) / (hist[k + 1][0] - hist[k][0])
            el_back = hist[k][1] + fb * (hist[k + 1][1] - hist[k][1])
            omega = (el_t - el_back) / 0.2
        else:
            omega = (el1 - el0) / (t1 - t0)
        return el_t, az0 + f * (az1 - az0), omega

    def _cb_range(self, msg: Range):
        if not self._collect:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        got = self._interp(t)
        if got is None:
            return
        el, az, omega = got
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        sq, status, shot_start = self._raw.get(key, (None, None, None))
        speed, pass_id = self._meta
        self._rows.append({
            "t": f"{t:.6f}",
            "d": "" if not math.isfinite(msg.range) else f"{msg.range:.4f}",
            "el_deg": f"{el:.4f}",
            "az_deg": f"{az:.4f}",
            "omega_meas": f"{omega:.3f}",
            "speed_cmd": f"{speed:.1f}",
            "dir": "1" if speed > 0 else "-1",
            "pass_id": str(pass_id),
            "signal_quality": "" if sq is None else f"{sq:.0f}",
            "status_code": "" if status is None else f"{status:.0f}",
            "shot_start": "" if shot_start is None else f"{shot_start:.6f}",
        })

    # --------------------------------------------------------------- ruch

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _send(self, az_deg, el_deg):
        by_name = {self._az_joint: math.radians(az_deg), self._el_joint: math.radians(el_deg)}
        msg = Float64MultiArray()
        msg.data = [float(by_name.get(j, self._positions.get(j, 0.0))) for j in self._order]
        self._pub_cmd.publish(msg)

    @staticmethod
    def _profile_time(d, vmax, accel):
        d = abs(d)
        if d < 1e-9:
            return 0.0
        if d <= vmax * vmax / accel:
            return 2.0 * math.sqrt(d / accel)
        return 2.0 * vmax / accel + (d - vmax * vmax / accel) / vmax

    def _send_vec(self, pub, by_name, fallback):
        msg = Float64MultiArray()
        msg.data = [float(by_name.get(j, fallback)) for j in self._order]
        pub.publish(msg)

    def _move_el(self, el_to, vmax) -> bool:
        """Przejazd elewacji PROFILEM SERWA: prędkość i przyspieszenie do
        kontrolerów profilu, potem JEDEN cel. Trajektorię liczy firmware z 1 kHz.

        Zastąpiło strumieniowanie pozycji co 20 ms (2026-09-17): przy włączonym
        profilu serwo planowało osobny mini-trapez przy każdym kroku, a głowica
        "klatkowała" - widoczne gołym okiem. Kąt do rekonstrukcji i tak bierzemy
        z enkoderów, więc to, KTO liczy trajektorię, nie zmienia pomiaru.
        """
        by_el = {self._el_joint: vmax, self._az_joint: self._travel}
        self._send_vec(self._pub_pvel, by_el, self._travel)
        self._send_vec(self._pub_pacc, {j: self._accel for j in self._order}, self._accel)
        # Profil musi dojść do serwa PRZED celem - serwo planuje trajektorię
        # w chwili przyjęcia Goal Position. Trzy cykle pętli z zapasem.
        if self._stop.wait(0.08):
            return False
        self._send(self._az, el_to)

        el_from = math.degrees(self._positions.get(self._el_joint, 0.0))
        expected = self._profile_time(el_to - el_from, vmax, self._accel)
        deadline = self._now() + expected * 1.5 + 2.0
        while self._now() < deadline:
            el = math.degrees(self._positions.get(self._el_joint, 0.0))
            if abs(el - el_to) <= 0.3:
                return True
            if self._stop.wait(0.02):
                return False
        self.get_logger().warn(f"Brak dojazdu elewacji do {el_to:.1f} st w {expected * 1.5 + 2.0:.1f} s")
        return True

    # --------------------------------------------------------------- przebieg

    def _run(self):
        while not self._positions and not self._stop.wait(0.1):
            pass
        if self._stop.is_set():
            return

        self.get_logger().info(
            f"Test rewersyjny: az={self._az:.1f} st, elewacja {self._el_min:.1f}..{self._el_max:.1f} st, "
            f"prędkości {self._speeds} st/s, po {self._passes} przejazdy w każdą stronę")

        for speed in self._speeds:
            if self._stop.is_set():
                break
            # Wyrównanie gęstości: przy dwukrotnie większej prędkości robimy
            # dwukrotnie więcej przejazdów, żeby każda prędkość miała podobną
            # liczbę punktów na stopień. Bez tego "Delta rośnie z prędkością"
            # może być artefaktem rzednącej siatki (błąd popełniony 2026-09-05).
            n = self._passes
            if self._equalize and self._speeds:
                n = max(1, int(round(self._passes * speed / min(self._speeds))))

            self._collect = False
            if not self._move_el(self._el_min, self._travel):
                break
            if self._stop.wait(self._settle):
                break

            for k in range(n):
                for direction, target in ((+1.0, self._el_max), (-1.0, self._el_min)):
                    if self._stop.is_set():
                        break
                    self._meta = (direction * speed, k)
                    self._collect = True
                    ok = self._move_el(target, speed)
                    self._collect = False
                    if not ok:
                        break
                    if self._stop.wait(0.3):
                        break
            self.get_logger().info(
                f"{speed:.0f} st/s: {n} par przejazdów gotowe, razem {len(self._rows)} pomiarów")

        self._collect = False
        self._move_el(0.0, self._travel)
        self._save()
        # Koniec testu kończy proces. Bez tego węzeł kręcił spin() w nieskończoność
        # po zapisie i skrypt uruchamiający musiał go zabijać ręcznie (2026-09-17).
        self.done.set()

    def _save(self):
        if not self._rows:
            self.get_logger().error("Brak pomiarów - nie ma czego zapisać")
            return
        os.makedirs(self._out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self._out_dir, f"{self._prefix}_{ts}.csv")
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self._rows[0].keys()))
            w.writeheader()
            w.writerows(self._rows)
        n_ok = sum(1 for r in self._rows if r["d"])
        self.get_logger().info(
            f"Zapisano {len(self._rows)} pomiarów ({n_ok} z odległością): {path}")
        self.get_logger().info("Analiza: scripts/anchor_from_rev.py <plik.csv>")

    def stop(self):
        self._stop.set()


def main():
    rclpy.init()
    node = AnchorRevTest()
    try:
        while rclpy.ok() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
