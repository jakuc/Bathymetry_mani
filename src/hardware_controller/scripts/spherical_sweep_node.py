#!/usr/bin/env python3
"""
spherical_sweep_node - sweep sferyczny obiema osiami głowicy, na realnym sprzęcie.

Przechodzi siatkę azymut x elewacja metodą "dojedź - ustój - zbieraj", a nie
ciągłym przelotem. To nie jest ostrożność: dalmierz Sharp jest analogowy i wolny,
a firmware liczy jeszcze medianę z okna ~50 ms, więc w ruchu każdy punkt
rozmazuje się po łuku i chmura wychodzi rozmyta w kierunku obrotu. W bezruchu
to samo uśrednianie tylko zbija szum.

Publikuje:
  /forward_position_controller/commands  (Float64MultiArray) - pozycje jointów [rad]
  /sweep/collecting                      (Bool)   - true, gdy głowica stoi i wolno zbierać
  /sweep/state                           (String) - "running" / "done"

Subskrybuje:
  /joint_states                          (JointState) - do wykrycia dojazdu

Parametry:
  azimuth_joint    (str, xm540_joint_z) - nazwa jointa azymutu (człon 1, serwo ID 2)
  elevation_joint  (str, xm540_joint)   - nazwa jointa elewacji (głowica, serwo ID 1)
  joint_order      (str[], [xm540_joint, xm540_joint_z]) - kolejność w komendzie;
                   MUSI odpowiadać liście `joints` forward_position_controller,
                   bo kontroler dostaje goły wektor bez nazw i nie ma jak wykryć
                   zamiany osi - objawem byłby sweep obrócony o 90 stopni.
  az_min_deg/az_max_deg/az_step_deg      - siatka azymutu
  el_min_deg/el_max_deg/el_step_deg      - siatka elewacji
  settle_time      (float, 0.4) - ile czekać po dojeździe, zanim zaczniemy zbierać [s]
  dwell_time       (float, 0.4) - ile zbierać w punkcie siatki [s]
  tolerance_deg    (float, 0.5) - próg uznania pozycji za osiągniętą
  arrival_timeout  (float, 5.0) - po tylu sekundach jedziemy dalej mimo braku dojazdu [s]
  serpentine       (bool, True) - co drugi wiersz w odwrotną stronę (krótsza droga)
  return_to_zero   (bool, True) - czy wrócić do zera po sweepie i przy zamknięciu
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


def _grid(lo, hi, step):
    """Siatka od lo do hi włącznie, z krokiem step (odporna na hi < lo i step <= 0)."""
    if step <= 0.0:
        raise ValueError("krok siatki musi być dodatni")
    if hi < lo:
        lo, hi = hi, lo
    # floor, nie round: przy kroku niedzielącym zakresu (np. -15..15 co 4)
    # zaokrąglenie w górę wypchnęłoby ostatni punkt POZA zadany zakres, czyli
    # głowica pojechałaby dalej, niż kazano. Zamiast tego domykamy prawy koniec
    # jawnie, żeby skan sięgał dokładnie do hi.
    n = int(math.floor((hi - lo) / step + 1e-9))
    pts = [lo + i * step for i in range(n + 1)]
    if hi - pts[-1] > 1e-9:
        pts.append(hi)
    return pts


class SphericalSweepNode(Node):
    def __init__(self):
        super().__init__("spherical_sweep_node")

        self.declare_parameter("azimuth_joint", "xm540_joint_z")
        self.declare_parameter("elevation_joint", "xm540_joint")
        self.declare_parameter("joint_order", ["xm540_joint", "xm540_joint_z"])

        self.declare_parameter("az_min_deg", -15.0)
        self.declare_parameter("az_max_deg", 15.0)
        self.declare_parameter("az_step_deg", 2.0)
        self.declare_parameter("el_min_deg", -15.0)
        self.declare_parameter("el_max_deg", 15.0)
        self.declare_parameter("el_step_deg", 2.0)

        self.declare_parameter("settle_time", 0.4)
        self.declare_parameter("dwell_time", 0.4)
        self.declare_parameter("tolerance_deg", 0.5)
        self.declare_parameter("arrival_timeout", 5.0)
        self.declare_parameter("serpentine", True)
        self.declare_parameter("return_to_zero", True)

        self._az_joint = self.get_parameter("azimuth_joint").value
        self._el_joint = self.get_parameter("elevation_joint").value
        self._order = list(self.get_parameter("joint_order").value)

        for name in (self._az_joint, self._el_joint):
            if name not in self._order:
                raise RuntimeError(
                    f"joint '{name}' nie występuje w joint_order={self._order} - "
                    f"komenda trafiłaby w nieistniejącą pozycję wektora")

        self._settle = self.get_parameter("settle_time").value
        self._dwell = self.get_parameter("dwell_time").value
        self._tol = math.radians(self.get_parameter("tolerance_deg").value)
        self._timeout = self.get_parameter("arrival_timeout").value
        self._return_to_zero = self.get_parameter("return_to_zero").value

        az = _grid(self.get_parameter("az_min_deg").value,
                   self.get_parameter("az_max_deg").value,
                   self.get_parameter("az_step_deg").value)
        el = _grid(self.get_parameter("el_min_deg").value,
                   self.get_parameter("el_max_deg").value,
                   self.get_parameter("el_step_deg").value)

        serpentine = self.get_parameter("serpentine").value
        self._points = []
        for i, a in enumerate(az):
            row = el if (not serpentine or i % 2 == 0) else list(reversed(el))
            for e in row:
                self._points.append((a, e))

        self._positions = {}
        self._stop = threading.Event()

        self._pub_cmd = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)
        self._pub_collecting = self.create_publisher(Bool, "/sweep/collecting", 10)
        # Stan sweepu z QoS transient_local: kolektor, który wstanie później,
        # i tak dostanie ostatnią wartość - inaczej mógłby przegapić "done"
        # i nigdy nie zapisać chmury.
        self._pub_state = self.create_publisher(
            String, "/sweep/state", QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))

        self.create_subscription(JointState, "/joint_states", self._cb_joints, 10)

        total = len(self._points)
        per_point = self._settle + self._dwell
        self.get_logger().info(
            f"Sweep sferyczny: azymut {az[0]:.1f}..{az[-1]:.1f} st ({len(az)} poz.), "
            f"elewacja {el[0]:.1f}..{el[-1]:.1f} st ({len(el)} poz.), "
            f"razem {total} punktów")
        self.get_logger().info(
            f"Szacowany czas: {total * per_point / 60.0:.1f} min "
            f"(bez czasu dojazdu; {per_point:.2f} s na punkt)")

        threading.Thread(target=self._run, daemon=True).start()

    # ------------------------------------------------------------------ joints

    def _cb_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = pos

    def _at_target(self, az_rad, el_rad) -> bool:
        a = self._positions.get(self._az_joint)
        e = self._positions.get(self._el_joint)
        if a is None or e is None:
            return False
        return abs(a - az_rad) <= self._tol and abs(e - el_rad) <= self._tol

    # ------------------------------------------------------------------ ruch

    def _send(self, az_rad, el_rad):
        by_name = {self._az_joint: az_rad, self._el_joint: el_rad}
        msg = Float64MultiArray()
        # Pozycje jointów spoza pary az/el (gdyby kontroler miał ich więcej)
        # zostawiamy tam, gdzie są - nie wymyślamy im wartości.
        msg.data = [float(by_name.get(j, self._positions.get(j, 0.0))) for j in self._order]
        self._pub_cmd.publish(msg)

    def _set_collecting(self, value: bool):
        self._pub_collecting.publish(Bool(data=value))

    def _sleep(self, seconds: float) -> bool:
        """Przerywalny sleep. Zwraca False, gdy w międzyczasie kazano się zatrzymać."""
        return not self._stop.wait(seconds)

    def _goto(self, az_deg, el_deg) -> bool:
        az_rad, el_rad = math.radians(az_deg), math.radians(el_deg)
        self._send(az_rad, el_rad)

        deadline = self.get_clock().now().nanoseconds * 1e-9 + self._timeout
        while not self._at_target(az_rad, el_rad):
            if not self._sleep(0.01):
                return False
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                self.get_logger().warn(
                    f"Brak dojazdu do (az={az_deg:.1f}, el={el_deg:.1f}) w {self._timeout:.1f} s "
                    f"- jadę dalej. Sprawdź limity w EEPROM serwa albo profil prędkości.",
                    throttle_duration_sec=5.0)
                return True
        return True

    # ------------------------------------------------------------------- sweep

    def _run(self):
        # Kontroler musi wstać i podać pierwsze /joint_states, zanim zaczniemy
        # cokolwiek liczyć na temat dojazdu.
        while not self._positions and self._sleep(0.1):
            pass
        if self._stop.is_set():
            return

        self._set_collecting(False)
        self._pub_state.publish(String(data="running"))
        self.get_logger().info("Sweep start.")

        done = 0
        for az_deg, el_deg in self._points:
            if self._stop.is_set():
                break

            self._set_collecting(False)
            if not self._goto(az_deg, el_deg):
                break

            # Osobno dojazd i uspokojenie: po dotarciu w tolerancję konstrukcja
            # jeszcze drga, a Sharp uśrednia okno wstecz - zbieranie od razu
            # zaciągnęłoby ogon poprzedniej pozycji.
            if not self._sleep(self._settle):
                break

            self._set_collecting(True)
            if not self._sleep(self._dwell):
                break
            self._set_collecting(False)

            done += 1
            if done % 25 == 0 or done == len(self._points):
                self.get_logger().info(f"Postęp: {done}/{len(self._points)} punktów")

        self._set_collecting(False)

        if self._return_to_zero and not self._stop.is_set():
            self.get_logger().info("Powrót do zera.")
            self._goto(0.0, 0.0)

        state = "done" if not self._stop.is_set() else "aborted"
        self._pub_state.publish(String(data=state))
        self.get_logger().info(f"Sweep zakończony ({done}/{len(self._points)} punktów).")

    def stop(self):
        self._stop.set()


def main():
    rclpy.init()
    node = SphericalSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
