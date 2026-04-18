"""
operator_panel – Interaktywny panel operatora do sterowania misją.

Uruchomienie (w nowym terminalu, po ros2 launch xm540_bringup isaac.launch.py):
    ros2 run xm540_bringup operator_panel

Panel wyświetla aktualny status misji i umożliwia:
  [1] Pomiar batymetryczny (wszystkie waypointy, pionowy sonar)
  [a] Przerwij misję
  [q] Wyjście
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

HEADER = """
╔══════════════════════════════════════════════╗
║       PANEL OPERATORA — Pomiar batymetrii    ║
╚══════════════════════════════════════════════╝"""

MENU = """
  [1]  Pomiar batymetryczny  (wszystkie waypointy, sonar pionowy)
  [2]  Sweep                 (wybrane punkty, skan stożkowy)
  [a]  Przerwij misję
  [c]  Wyczyść chmurę punktów
  [q]  Wyjście
"""


class OperatorPanel(Node):
    def __init__(self):
        super().__init__("operator_panel")
        self._status = "IDLE"
        self._cli_baseline = self.create_client(Trigger, "/mission/start_baseline")
        self._cli_sweep    = self.create_client(Trigger, "/mission/start_sweep")
        self._cli_abort    = self.create_client(Trigger, "/mission/abort")
        self._cli_clear    = self.create_client(Empty,   "/scan_collector_node/clear")
        self.create_subscription(String, "/mission/status", self._cb_status, 10)

    def _cb_status(self, msg: String) -> None:
        self._status = msg.data
        # Wypisz update statusu bezpośrednio — widoczne między liniami menu
        print(f"\n  ▶ Status: {self._status}", flush=True)

    def _call(self, client, label: str) -> None:
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"  ✗ Serwis niedostępny ({label}). Czy supervisor działa?")
            return
        srv_type = client.srv_type
        future = client.call_async(srv_type.Request())
        future.add_done_callback(self._on_service_response)

    def _on_service_response(self, future) -> None:
        try:
            res = future.result()
            if hasattr(res, "success"):
                icon = "✓" if res.success else "✗"
                msg  = getattr(res, "message", "")
                print(f"\n  {icon} {msg}", flush=True)
            else:
                print(f"\n  ✓", flush=True)
        except Exception as e:
            print(f"\n  ✗ Błąd serwisu: {e}", flush=True)

    def run(self) -> None:
        print(HEADER)
        print(f"  Status: {self._status}")
        print(MENU)

        while rclpy.ok():
            try:
                choice = input("  > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == "1":
                print("  Startuję pomiar batymetryczny...")
                self._call(self._cli_baseline, "/mission/start_baseline")

            elif choice == "2":
                print("  Startuję sweep...")
                self._call(self._cli_sweep, "/mission/start_sweep")

            elif choice == "a":
                print("  Przerywam misję...")
                self._call(self._cli_abort, "/mission/abort")

            elif choice == "c":
                print("  Czyszczę chmurę punktów...")
                self._call(self._cli_clear, "/scan_collector_node/clear")

            elif choice == "q":
                break

            else:
                print(MENU)


def main():
    rclpy.init()
    panel = OperatorPanel()

    # ROS spin w tle — odbiera /mission/status bez blokowania input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(panel,), daemon=True)
    spin_thread.start()

    try:
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        panel.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
