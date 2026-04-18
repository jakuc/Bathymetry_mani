"""
Driver node: komunikacja z serwem XM540-W270 przez RS485.

Subskrybuje:
  /servo/goal_position  (std_msgs/Float64)  – zadana pozycja [°]

Publikuje:
  /joint_states         (sensor_msgs/JointState)
  /servo/temperature    (std_msgs/Float64)  – [°C]
  /servo/current        (std_msgs/Float64)  – [raw units]
"""

import fcntl
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import dynamixel_sdk as dxl
from . import config


class DynamixelNode(Node):
    def __init__(self):
        super().__init__("dynamixel_node")

        self._open_port()

        self._set_operating_mode(config.MODE_POSITION)
        self._write4(config.ADDR_PROFILE_VELOCITY, 0)
        self._write4(config.ADDR_PROFILE_ACCELERATION, 0)
        self._write1(config.ADDR_TORQUE_ENABLE, 1)

        self.pub_js   = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_temp = self.create_publisher(Float64, "/servo/temperature", 10)
        self.pub_curr = self.create_publisher(Float64, "/servo/current", 10)

        self.create_subscription(Float64, "/servo/goal_position",
                                 self._cb_goal, 10)

        self.create_timer(0.02, self._publish_state)   # 50 Hz

        self.get_logger().info("DynamixelNode gotowy.")

    # ------------------------------------------------------------------
    # Callback – zadana pozycja
    # ------------------------------------------------------------------

    def _cb_goal(self, msg: Float64):
        raw = self._deg_to_raw(msg.data)
        self._write4(config.ADDR_GOAL_POSITION, raw)

    # ------------------------------------------------------------------
    # Publikacja stanu (50 Hz)
    # ------------------------------------------------------------------

    def _publish_state(self):
        pos_raw = self._read4(config.ADDR_PRESENT_POSITION)
        vel_raw = self._read4(config.ADDR_PRESENT_VELOCITY)
        cur_raw = self._read2(config.ADDR_PRESENT_CURRENT)
        tmp_raw = self._read1(config.ADDR_PRESENT_TEMPERATURE)

        pos_deg = self._raw_to_deg(pos_raw)
        pos_rad = pos_deg * 3.14159265 / 180.0

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ["xm540_joint"]
        js.position = [pos_rad]
        js.velocity = [float(vel_raw)]
        js.effort   = [float(cur_raw)]
        self.pub_js.publish(js)

        t = Float64(); t.data = float(tmp_raw)
        self.pub_temp.publish(t)

        c = Float64(); c.data = float(cur_raw)
        self.pub_curr.publish(c)

    # ------------------------------------------------------------------
    # Konwersje
    # ------------------------------------------------------------------

    def _deg_to_raw(self, deg: float) -> int:
        return int(config.CENTER_RAW + deg * config.ENCODER_RESOLUTION / 360.0)

    def _raw_to_deg(self, raw: int) -> float:
        return (raw - config.CENTER_RAW) * 360.0 / config.ENCODER_RESOLUTION

    # ------------------------------------------------------------------
    # Niskopoziomowe operacje na rejestrach
    # ------------------------------------------------------------------

    def _open_port(self):
        port = config.DEVICE_PORT

        # Sprawdź czy port w ogóle istnieje
        if not os.path.exists(port):
            raise IOError(
                f"Port {port} nie istnieje. "
                f"Sprawdź czy U2D2 jest podłączone (ls /dev/ttyUSB*)."
            )

        # Wyłączny lock na pliku urządzenia – blokuje drugi proces natychmiast
        try:
            self._lock_fd = open(port, "rb+", buffering=0)
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise IOError(
                f"Port {port} jest już używany przez inny proces. "
                f"Sprawdź czy poprzedni dynamixel_node nie działa: "
                f"'pkill -f dynamixel_node'"
            )

        self.ph = dxl.PortHandler(port)
        self.pk = dxl.PacketHandler(config.PROTOCOL)
        if not self.ph.openPort():
            raise IOError(f"Nie można otworzyć portu {port}")
        if not self.ph.setBaudRate(config.BAUD_RATE):
            raise IOError(f"Nie można ustawić baud rate {config.BAUD_RATE}")

    def _set_operating_mode(self, mode: int):
        self._write1(config.ADDR_TORQUE_ENABLE, 0)
        self._write1(config.ADDR_OPERATING_MODE, mode)

    def _write1(self, addr, val):
        r, e = self.pk.write1ByteTxRx(self.ph, config.SERVO_ID, addr, val)
        self._chk(r, e)

    def _write4(self, addr, val):
        r, e = self.pk.write4ByteTxRx(self.ph, config.SERVO_ID, addr, val)
        self._chk(r, e)

    def _read1(self, addr) -> int:
        v, r, e = self.pk.read1ByteTxRx(self.ph, config.SERVO_ID, addr)
        self._chk(r, e); return v

    def _read2(self, addr) -> int:
        v, r, e = self.pk.read2ByteTxRx(self.ph, config.SERVO_ID, addr)
        self._chk(r, e); return v

    def _read4(self, addr) -> int:
        v, r, e = self.pk.read4ByteTxRx(self.ph, config.SERVO_ID, addr)
        self._chk(r, e); return v

    def _chk(self, result, error):
        if result != dxl.COMM_SUCCESS:
            msg = self.pk.getTxRxResult(result)
            self.get_logger().error(
                f"Błąd komunikacji z serwem ID={config.SERVO_ID}: {msg} "
                f"(port={config.DEVICE_PORT}, baud={config.BAUD_RATE})"
            )
        elif error != 0:
            self.get_logger().warn(
                f"Błąd pakietu serwa ID={config.SERVO_ID}: "
                f"{self.pk.getRxPacketError(error)}"
            )

    def destroy_node(self):
        self._write1(config.ADDR_TORQUE_ENABLE, 0)
        self.ph.closePort()
        # zwolnij lock
        if hasattr(self, "_lock_fd"):
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            self._lock_fd.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DynamixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
