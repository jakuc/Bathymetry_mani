"""
batch_sweep_node – automatyczne sekwencyjne uruchamianie skanów sweep.

Skala jeziora jest odczytywana automatycznie z world_scale parametru
isaac_sim_node, więc nie trzeba jej podawać ręcznie.

Uruchomienie:
    ros2 run xm540_bringup batch_sweep_node \
        --ros-args \
        -p waypoints_dir:=/workspace/install/xm540_bringup/share/xm540_bringup/waypoints \
        -p step_degs:=[1.0,2.0] \
        -p n_points:=[1,2,3,4,5] \
        -p delay_between:=3.0

Kolejność: dla każdego step_deg, dla każdej liczby punktów → jeden sweep.
Po każdym skanie: czeka na IDLE, czyści chmurę, ustawia nowe parametry, startuje kolejny.
"""

import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from ament_index_python.packages import get_package_share_directory


class BatchSweepNode(Node):
    def __init__(self):
        super().__init__("batch_sweep_node")

        _default_wp_dir = os.path.join(get_package_share_directory("xm540_bringup"), "waypoints")
        self.declare_parameter("waypoints_dir", _default_wp_dir)
        self.declare_parameter("step_degs",     [2.0, 1.0])
        self.declare_parameter("n_points",      [1, 2, 3, 4, 5])
        self.declare_parameter("delay_between", 3.0)

        self._waypoints_dir = self.get_parameter("waypoints_dir").value
        self._step_degs     = self.get_parameter("step_degs").value
        self._n_points_list = self.get_parameter("n_points").value
        self._delay         = self.get_parameter("delay_between").value

        self._queue: list[tuple[str, float]] = []
        self._idx            = 0
        self._mission_active = False

        self._cli_sweep      = self.create_client(Trigger,        "/mission/start_sweep")
        self._cli_clear      = self.create_client(Empty,          "/scan_collector_node/clear")
        self._get_scale_cli  = self.create_client(GetParameters,  "/isaac_sim_node/get_parameters")
        self._set_params_cli = self.create_client(SetParameters,  "/mission_supervisor_node/set_parameters")

        self.create_subscription(String, "/mission/status", self._cb_status, 10)

        self.get_logger().info("Odczytuję world_scale z isaac_sim_node...")
        self.create_timer(2.0, self._read_scale)

    def _read_scale(self):
        self.destroy_timer(list(self.timers)[0])
        if not self._get_scale_cli.service_is_ready():
            self.get_logger().info("isaac_sim_node jeszcze nie gotowy, retry za 2s...")
            self.create_timer(2.0, self._read_scale)
            return
        req = GetParameters.Request()
        req.names = ["world_scale"]
        self._get_scale_cli.call_async(req).add_done_callback(self._on_scale_read)

    def _on_scale_read(self, future):
        try:
            result = future.result()
            world_scale = float(result.values[0].double_value)
        except Exception as e:
            self.get_logger().error(f"Nie udało się odczytać world_scale: {e}. Używam domyślnej x10.")
            world_scale = 10.0

        scale_int = int(round(world_scale))
        scale_str = f"x{scale_int}"
        self.get_logger().info(f"Wykryta skala: world_scale={world_scale} → '{scale_str}'")

        for step in self._step_degs:
            for n in self._n_points_list:
                fname = f"waypoints_sweep_{n}pt_{scale_str}.csv"
                fpath = f"{self._waypoints_dir}/{fname}"
                self._queue.append((fpath, float(step)))

        self.get_logger().info(
            f"Kolejka: {len(self._queue)} skanów "
            f"({len(self._n_points_list)} pliki × {len(self._step_degs)} kroki kątowe)"
        )
        for i, (f, s) in enumerate(self._queue):
            self.get_logger().info(f"  [{i+1}/{len(self._queue)}] step={s}°  {f}")

        self.create_timer(3.0, self._start_first)

    def _start_first(self):
        self.destroy_timer(list(self.timers)[0])
        self._run_next()

    def _cb_status(self, msg: String) -> None:
        if msg.data == "IDLE" and self._mission_active:
            self._mission_active = False
            self.get_logger().info("Misja zakończona. Kolejny skan za {:.0f}s...".format(self._delay))
            self.create_timer(self._delay, self._advance)

    def _advance(self):
        self.destroy_timer(list(self.timers)[0])
        self._idx += 1
        self._run_next()

    def _run_next(self):
        if self._idx >= len(self._queue):
            self.get_logger().info("Wszystkie skany zakończone.")
            return

        fpath, step_deg = self._queue[self._idx]
        self.get_logger().info(
            f"[{self._idx+1}/{len(self._queue)}] step={step_deg}°  {fpath}"
        )

        p1 = RclParameter()
        p1.name = "sweep_waypoints_file"
        p1.value = ParameterValue()
        p1.value.type = ParameterType.PARAMETER_STRING
        p1.value.string_value = fpath

        p2 = RclParameter()
        p2.name = "sweep_step_deg"
        p2.value = ParameterValue()
        p2.value.type = ParameterType.PARAMETER_DOUBLE
        p2.value.double_value = step_deg

        req = SetParameters.Request()
        req.parameters = [p1, p2]

        if not self._set_params_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("mission_supervisor_node/set_parameters niedostępny")
            return
        self._set_params_cli.call_async(req).add_done_callback(self._on_params_set)

    def _on_params_set(self, future):
        try:
            result = future.result()
            for r in result.results:
                if not r.successful:
                    self.get_logger().error(f"Nie udało się ustawić parametru: {r.reason}")
                    return
        except Exception as e:
            self.get_logger().error(f"Błąd ustawiania parametrów: {e}")
            return

        if not self._cli_clear.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Serwis clear niedostępny")
            return
        self._cli_clear.call_async(Empty.Request()).add_done_callback(self._on_cleared)

    def _on_cleared(self, future):
        if not self._cli_sweep.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Serwis start_sweep niedostępny")
            return
        self._cli_sweep.call_async(Trigger.Request()).add_done_callback(self._on_sweep_started)

    def _on_sweep_started(self, future):
        try:
            res = future.result()
            if res.success:
                self._mission_active = True
                self.get_logger().info(f"Skan {self._idx+1}/{len(self._queue)} uruchomiony.")
            else:
                self.get_logger().error(f"Nie udało się uruchomić sweepа: {res.message}")
        except Exception as e:
            self.get_logger().error(f"Błąd start_sweep: {e}")


def main():
    rclpy.init()
    node = BatchSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
