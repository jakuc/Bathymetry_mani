"""
sim_driver_node – symulator dwóch serwomechanizmów XM540 w Gazebo.

Serwisy:
  /set_orientation (SetOrientation) – oba jointy naraz
  /start_sweep     (StartSweep)     – automatyczny sweep wybranego jointu
"""

import math
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from xm540_interfaces.srv import SetOrientation, StartSweep

LIMIT             = math.pi / 2  # ±90° — fizyczny limit sprzętu
DEFAULT_MAX_VEL   = 2.69          # rad/s
DEFAULT_MAX_ACC   = 10.0          # rad/s²
DEFAULT_SONAR_HZ  = 20.0          # Hz


class SimDriverNode(Node):
    def __init__(self):
        super().__init__("sim_driver_node")

        # Stan obu jointów: [joint_z, joint_y] – aktualizowany z /joint_states (fizyka)
        self.current = [0.0, 0.0]
        self.goal    = [0.0, 0.0]

        # Wewnętrzny profil trapezoidalny – pozycja i prędkość komendy do Isaac Sim
        self._cmd     = [0.0, 0.0]
        self._cmd_vel = [0.0, 0.0]

        # Prędkość Y podczas sweep (None = używaj max_vel_rad_s)
        self._y_sweep_vel: float | None = None

        self.declare_parameter("max_vel_rad_s",  DEFAULT_MAX_VEL)
        self.declare_parameter("max_acc_rad_s2", DEFAULT_MAX_ACC)
        self.declare_parameter("sonar_rate_hz",  DEFAULT_SONAR_HZ)

        self._sweep_cancel = threading.Event()

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
        for name, pos in zip(msg.name, msg.position):
            if name == "xm540_joint_z":
                self.current[0] = pos
            elif name == "xm540_joint":
                self.current[1] = pos

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
        response.success = True
        response.message = (
            f"Skan ciągły: Y=±{range_deg:.0f}°, Z=±90°, "
            f"krok Z {step_deg:.1f}°, {n_z + 1} pasów, sonar 20 Hz"
        )
        return response

    # --- conical scan thread ---

    @staticmethod
    def _fmt_time(seconds: float) -> str:
        s = int(seconds)
        if s < 60:
            return f"{s}s"
        return f"{s // 60}m {s % 60:02d}s"

    def _sweep_done(self) -> None:
        self._pub_sweep_active.publish(Bool(data=False))

    def _conical_scan_thread(self, range_deg, step_deg, tolerance_deg, cancel):
        max_vel       = self.get_parameter("max_vel_rad_s").value
        max_acc       = self.get_parameter("max_acc_rad_s2").value
        sonar_rate_hz = self.get_parameter("sonar_rate_hz").value
        tol_rad       = math.radians(tolerance_deg)

        # Prędkość Y: step_deg na jeden okres sonara
        sweep_vel = min(math.radians(step_deg) * sonar_rate_hz, max_vel)
        self._y_sweep_vel = sweep_vel

        def wait_arrived(idx) -> bool:
            """Czeka aż profil _cmd[idx] dotrze do celu. Zwraca False jeśli anulowano."""
            while True:
                if cancel.is_set():
                    return False
                if abs(self._cmd[idx] - self.goal[idx]) <= tol_rad:
                    return True
                time.sleep(0.002)

        def trap_time(deg: float, v_max: float) -> float:
            d      = math.radians(abs(deg))
            d_crit = v_max * v_max / (2.0 * max_acc)
            if d <= d_crit:
                return 2.0 * math.sqrt(2.0 * d / max_acc)
            return v_max / max_acc + d / v_max

        z_angles  = [-90.0 + i * step_deg for i in range(int(round(180.0 / step_deg)) + 1)]
        n_total   = len(z_angles)
        est_total = (n_total * trap_time(2 * range_deg, sweep_vel)
                     + (n_total - 1) * trap_time(step_deg, max_vel))

        self.get_logger().info(
            f"Skan ciągły start: Y=±{range_deg:.0f}°, Z=±90°, "
            f"krok Z {step_deg:.1f}°, {n_total} pasów, "
            f"prędkość Y {math.degrees(sweep_vel):.1f}°/s. "
            f"Szacowany czas: {self._fmt_time(est_total)}."
        )
        scan_start = time.monotonic()

        def abort():
            self.get_logger().info("Skan przerwany.")
            self._y_sweep_vel = None
            self._sweep_done()

        # Prepozycja Y na początek pierwszego pasa
        self.goal[1] = math.radians(-range_deg)
        if not wait_arrived(1):
            abort()
            return

        for iz, z_deg in enumerate(z_angles):
            # Przesuń Z do pozycji pasa — sonar zbiera dane podczas ruchu
            self.goal[0] = math.radians(z_deg)
            if not wait_arrived(0):
                abort()
                return

            # Snake: parzyste pasy +range, nieparzyste -range
            y_end     = range_deg if iz % 2 == 0 else -range_deg
            elapsed   = time.monotonic() - scan_start
            remaining = est_total - elapsed
            self.get_logger().info(
                f"Pas {iz + 1:3d}/{n_total} ({100 * (iz + 1) // n_total:3d}%) "
                f"| Z={z_deg:+.1f}° | Y→{y_end:+.0f}° "
                f"| pozostało ~{self._fmt_time(max(0.0, remaining))}"
            )

            self.goal[1] = math.radians(y_end)
            if not wait_arrived(1):
                abort()
                return

        elapsed = time.monotonic() - scan_start
        self.get_logger().info(
            f"Skan zakończony. Czas: {self._fmt_time(elapsed)}. Powrót do 0°."
        )
        self._y_sweep_vel = None
        self.goal[0] = 0.0
        self.goal[1] = 0.0
        wait_arrived(0)
        wait_arrived(1)
        self.get_logger().info("Powrót zakończony.")
        self._sweep_done()

    # --- timer 100 Hz ---

    def _tick(self):
        max_vel = self.get_parameter("max_vel_rad_s").value
        max_acc = self.get_parameter("max_acc_rad_s2").value
        dt = 0.01
        vel_limits = [max_vel, self._y_sweep_vel if self._y_sweep_vel is not None else max_vel]
        for i in range(2):
            delta = self.goal[i] - self._cmd[i]
            if abs(delta) < 1e-9:
                self._cmd_vel[i] = 0.0
                continue

            v_brake  = math.sqrt(2.0 * max_acc * abs(delta))
            v_target = math.copysign(min(vel_limits[i], v_brake), delta)

            dv_max = max_acc * dt
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
