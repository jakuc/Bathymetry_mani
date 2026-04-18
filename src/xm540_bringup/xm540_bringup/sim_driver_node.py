"""
sim_driver_node – symulator dwóch serwomechanizmów XM540 w Gazebo.

Serwisy:
  /set_orientation (SetOrientation) – oba jointy naraz
  /start_sweep     (StartSweep)     – automatyczny sweep wybranego jointu
"""

import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from xm540_interfaces.srv import SetOrientation, StartSweep

LIMIT       = math.pi / 2        # ±90°
MAX_VEL     = 2.69               # rad/s
MAX_ACC     = 10.0               # rad/s² – przyspieszenie/hamowanie (profil trapezoidalny)
VEL_STOPPED = math.radians(2.0)  # rad/s – próg "joint stoi" w warunku sweep


class SimDriverNode(Node):
    def __init__(self):
        super().__init__("sim_driver_node")

        # Stan obu jointów: [joint_z, joint_y] – aktualizowany z /sim/encoders (fizyka)
        self.current  = [0.0, 0.0]
        self.velocity = [0.0, 0.0]   # estymowana prędkość z różnicy enkoderów [rad/s]
        self.goal     = [0.0, 0.0]

        # Wewnętrzny profil trapezoidalny – pozycja i prędkość komendy do Isaac Sim
        self._cmd     = [0.0, 0.0]
        self._cmd_vel = [0.0, 0.0]

        # Do estymacji prędkości z enkoderów
        self._last_enc_stamp = 0.0

        # Anulowanie bieżącego sweepa
        self._sweep_cancel = threading.Event()

        # Event sygnalizujący nowe dane enkoderów — sweep thread czeka na ten event
        # zamiast sleepować, dzięki czemu działa poprawnie przy przyspieszeniu symulacji
        self._enc_event = threading.Event()

        self.pub_gz_y = self.create_publisher(Float64, "/xm540_joint/cmd_pos",   10)
        self.pub_gz_z = self.create_publisher(Float64, "/xm540_joint_z/cmd_pos", 10)
        self._pub_sweep_active = self.create_publisher(Bool, "/sweep_active", 10)

        self.create_subscription(Float64,    "/servo/goal_position", self._cb_goal_y,   10)
        self.create_subscription(JointState, "/joint_states",        self._cb_encoders, 10)

        self.create_service(SetOrientation, "set_orientation", self._srv_set_orientation)
        self.create_service(StartSweep,     "start_sweep",     self._srv_start_sweep)

        self.create_timer(0.01, self._tick)

        self.get_logger().info("SimDriverNode gotowy (2 jointy: joint_z / joint_y).")

    # --- subskrypcje ---

    def _cb_goal_y(self, msg: Float64):
        self._sweep_cancel.set()
        self.goal[1] = math.radians(msg.data)

    def _cb_encoders(self, msg: JointState) -> None:
        """Odbiera /joint_states z isaac_sim i aktualizuje stan wewnętrzny (dla profilu i sweep)."""
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = stamp_sec - self._last_enc_stamp

        prev = list(self.current)
        for name, pos in zip(msg.name, msg.position):
            if name == "xm540_joint_z":
                self.current[0] = pos
            elif name == "xm540_joint":
                self.current[1] = pos

        if self._last_enc_stamp > 0.0 and 0.0 < dt < 0.5:
            self.velocity[0] = (self.current[0] - prev[0]) / dt
            self.velocity[1] = (self.current[1] - prev[1]) / dt
        self._last_enc_stamp = stamp_sec
        self._enc_event.set()

    # --- serwisy ---

    def _srv_set_orientation(self, request, response):
        rad_z = math.radians(float(request.joint_z_deg))
        rad_y = math.radians(float(request.joint_y_deg))
        errors = []
        if not (-LIMIT <= rad_z <= LIMIT):
            errors.append(f"joint_z {request.joint_z_deg:.1f}° poza zakresem ±90°")
        if not (-LIMIT <= rad_y <= LIMIT):
            errors.append(f"joint_y {request.joint_y_deg:.1f}° poza zakresem ±90°")
        if errors:
            response.success = False
            response.message = ", ".join(errors)
            return response
        self._sweep_cancel.set()
        self.goal[0] = rad_z
        self.goal[1] = rad_y
        response.success = True
        response.message = f"Cel: joint_z={request.joint_z_deg:.1f}°, joint_y={request.joint_y_deg:.1f}°"
        return response

    def _srv_start_sweep(self, request, response):
        range_deg     = float(request.range_deg)
        step_deg      = max(0.1, float(request.step_deg))
        tolerance_deg = float(request.tolerance_deg) if request.tolerance_deg > 0 else 0.5

        if not (0 < range_deg <= 90):
            response.success = False
            response.message = f"range_deg={range_deg:.1f} musi być w zakresie (0, 90]"
            return response

        self._sweep_cancel.set()
        self._sweep_cancel = threading.Event()
        cancel = self._sweep_cancel

        self._pub_sweep_active.publish(Bool(data=True))
        threading.Thread(
            target=self._conical_scan_thread,
            args=(range_deg, step_deg, tolerance_deg, cancel),
            daemon=True,
        ).start()

        n_z = int(round(180.0 / step_deg))
        n_y = int(round(2 * range_deg / step_deg))
        response.success = True
        response.message = (
            f"Skan stożkowy: Y=±{range_deg:.0f}°, Z=±90°, "
            f"krok {step_deg:.1f}°, {n_z} pasów Y × {n_y} kroków, 20 Hz"
        )
        return response

    # --- conical scan thread ---

    SCAN_PERIOD = 0.05   # 20 Hz – częstotliwość czujnika

    @staticmethod
    def _fmt_time(seconds: float) -> str:
        s = int(seconds)
        if s < 60:
            return f"{s}s"
        return f"{s // 60}m {s % 60:02d}s"

    def _sweep_done(self) -> None:
        self._pub_sweep_active.publish(Bool(data=False))

    def _conical_scan_thread(self, range_deg, step_deg, tolerance_deg, cancel):
        tol_rad = math.radians(tolerance_deg)

        # Pozycje Z: od -90° do +90°, krok step_deg
        n_z      = int(round(180.0 / step_deg))
        z_angles = [-90.0 + i * step_deg for i in range(n_z + 1)]

        # Pozycje Y: od -range do +range, krok step_deg
        n_y        = int(round(range_deg / step_deg))
        y_forward  = [i * step_deg for i in range(-n_y, n_y + 1)]
        y_backward = list(reversed(y_forward))

        # Szacowany czas ruchu trapezoidalnego o daną odległość kątową
        def trap_time(deg: float) -> float:
            d      = math.radians(deg)
            d_crit = MAX_VEL * MAX_VEL / (2.0 * MAX_ACC)
            if d <= d_crit:
                return 2.0 * math.sqrt(2.0 * d / MAX_ACC)   # profil trójkątny
            return MAX_VEL / MAX_ACC + d / MAX_VEL           # profil trapezoidalny

        steps_per_strip  = len(y_forward)
        y_time_per_strip = steps_per_strip * trap_time(step_deg)
        z_step_time      = trap_time(step_deg)
        n_total          = len(z_angles)
        est_total        = n_total * y_time_per_strip + (n_total - 1) * z_step_time

        self.get_logger().info(
            f"Skan stożkowy start: Y=±{range_deg:.0f}°, Z=±90°, "
            f"krok {step_deg:.1f}°, {n_total} pasów × {steps_per_strip} kroków. "
            f"Szacowany czas: {self._fmt_time(est_total)}."
        )
        scan_start = self.get_clock().now()

        for iz, z_deg in enumerate(z_angles):
            if cancel.is_set():
                self.get_logger().info("Skan przerwany.")
                self._sweep_done()
                return

            # Przesuń Z i poczekaj aż joint stoi w tolerancji (pozycja + prędkość)
            self.goal[0] = math.radians(z_deg)
            while True:
                if cancel.is_set():
                    self.get_logger().info("Skan przerwany.")
                    self._sweep_done()
                    return
                if (abs(self.current[0] - self.goal[0]) <= tol_rad
                        and abs(self.velocity[0]) <= VEL_STOPPED):
                    break
                self._enc_event.wait()
                self._enc_event.clear()

            # Snake pattern: parzyste pasy w przód, nieparzyste wstecz
            y_angles = y_forward if iz % 2 == 0 else y_backward
            kierunek  = f"-{range_deg:.0f}°→+{range_deg:.0f}°" if iz % 2 == 0 \
                        else f"+{range_deg:.0f}°→-{range_deg:.0f}°"
            elapsed   = (self.get_clock().now() - scan_start).nanoseconds * 1e-9
            remaining = est_total - elapsed
            self.get_logger().info(
                f"Pas {iz + 1:3d}/{n_total} ({100 * (iz + 1) // n_total:3d}%) "
                f"| Z={z_deg:+.1f}° | Y: {kierunek} "
                f"| pozostało ~{self._fmt_time(max(0.0, remaining))}"
            )

            for y_deg in y_angles:
                if cancel.is_set():
                    self.get_logger().info("Skan przerwany.")
                    self._sweep_done()
                    return

                # Zadaj cel Y i czekaj aż joint stoi w tolerancji (pozycja + prędkość)
                self.goal[1] = math.radians(float(y_deg))
                while True:
                    if cancel.is_set():
                        self.get_logger().info("Skan przerwany.")
                        self._sweep_done()
                        return
                    if (abs(self.current[1] - self.goal[1]) <= tol_rad
                            and abs(self.velocity[1]) <= VEL_STOPPED):
                        break
                    self._enc_event.wait()
                    self._enc_event.clear()

        elapsed = (self.get_clock().now() - scan_start).nanoseconds * 1e-9
        self.get_logger().info(
            f"Skan stożkowy zakończony. Czas symulacji: {self._fmt_time(elapsed)}. "
            f"Powrót do pozycji 0°, 0°."
        )

        # Powrót do pozycji zerowej
        self.goal[0] = 0.0
        self.goal[1] = 0.0
        while True:
            if cancel.is_set():
                return
            if abs(self.current[0]) <= tol_rad and abs(self.current[1]) <= tol_rad:
                break
            self._enc_event.wait()
            self._enc_event.clear()
        self.get_logger().info("Powrót zakończony.")
        self._sweep_done()

    # --- timer 100 Hz ---

    def _tick(self):
        dt = 0.01
        for i in range(2):
            delta = self.goal[i] - self._cmd[i]
            if abs(delta) < 1e-9:
                self._cmd_vel[i] = 0.0
                continue

            # Prędkość hamowania: v = sqrt(2 * a * |delta|), ograniczona do MAX_VEL
            v_brake  = math.sqrt(2.0 * MAX_ACC * abs(delta))
            v_target = math.copysign(min(MAX_VEL, v_brake), delta)

            # Rampa przyspieszenia/hamowania
            dv_max = MAX_ACC * dt
            if self._cmd_vel[i] < v_target:
                self._cmd_vel[i] = min(self._cmd_vel[i] + dv_max, v_target)
            else:
                self._cmd_vel[i] = max(self._cmd_vel[i] - dv_max, v_target)

            self._cmd[i] += self._cmd_vel[i] * dt

            # Nie przekraczaj celu
            if math.copysign(1.0, self.goal[i] - self._cmd[i]) != math.copysign(1.0, delta):
                self._cmd[i]     = self.goal[i]
                self._cmd_vel[i] = 0.0

        cmd_z = Float64(); cmd_z.data = self._cmd[0]
        cmd_y = Float64(); cmd_y.data = self._cmd[1]
        self.pub_gz_z.publish(cmd_z)
        self.pub_gz_y.publish(cmd_y)


def main():
    rclpy.init()
    node = SimDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
