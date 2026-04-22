"""
mission_supervisor_node – Automatyczny pomiar batymetryczny.

Dwa scenariusze:
  baseline  – łódka płynie ciągłą trasą przez waypoints, sonar pinkuje cały czas.
              Isaac Sim utrzymuje kolejkę waypointów; supervisor uzupełnia ją po
              każdym /boat_arrived (sliding window = boat_buffer_size).
  sweep     – łódka zatrzymuje się w każdym waypoincie, pełny skan stożkowy, jedzie dalej.

Maszyna stanów (baseline):
  IDLE → CRUISING → DONE → IDLE
  (postęp śledzony przez _wp_sent / _wp_completed; stan CRUISING trwa przez całą misję)

Maszyna stanów (sweep):
  IDLE → SEND_GOAL → MOVING → STABILIZE → SWEEP_START → SWEEPING → NEXT → DONE → IDLE

Serwisy (oferowane):
  /mission/start_baseline  (std_srvs/Trigger)
  /mission/start_sweep     (std_srvs/Trigger)
  /mission/abort           (std_srvs/Trigger)

Topiki:
  /mission/status  (std_msgs/String) – postęp, odczytywany przez operator_panel

Konfiguracja: config/mission.yaml (ładowany przez isaac.launch.py)
"""

import csv
import os
import pathlib
import time
from enum import Enum, auto

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from xm540_interfaces.srv import SetBoatPose, StartSweep

_PKG_SHARE = get_package_share_directory("xm540_bringup")


class State(Enum):
    IDLE          = auto()
    CRUISING      = auto()   # baseline: łódka jedzie ciągle, supervisor uzupełnia kolejkę
    SEND_GOAL     = auto()
    MOVING        = auto()
    STABILIZE     = auto()
    WAITING_SONAR = auto()
    SWEEP_START   = auto()
    SWEEPING      = auto()
    NEXT          = auto()
    DONE          = auto()


class MissionSupervisorNode(Node):
    def __init__(self):
        super().__init__("mission_supervisor_node")

        # Parametry — scenariusz 1
        self.declare_parameter("waypoints_file",    os.path.join(_PKG_SHARE, "waypoints.csv"))
        self.declare_parameter("boat_buffer_size",  5)   # ile waypointów z góry w kolejce Isaaca
        self.declare_parameter("stabilize_time",    0.5)
        self.declare_parameter("n_readings",        1)
        self.declare_parameter("progress_interval", 0)  # 0 = tryb procentowy (co 5%)

        # Parametry — scenariusz 2
        self.declare_parameter("sweep_waypoints_file", os.path.join(_PKG_SHARE, "sweep_waypoints.csv"))
        self.declare_parameter("sweep_stabilize_time", 1.0)
        self.declare_parameter("sweep_range_deg",      45.0)
        self.declare_parameter("sweep_step_deg",       5.0)
        self.declare_parameter("sweep_tolerance_deg",  0.5)

        # Stan wewnętrzny
        self._state        = State.IDLE
        self._scenario     = "baseline"
        self._waypoints: list[tuple[float, float, float]] = []
        self._wp_idx       = 0    # sweep: aktualny waypoint
        self._wp_sent      = 0    # baseline: ile waypointów wysłano do kolejki Isaaca
        self._wp_completed = 0    # baseline: ile waypointów łódka ukończyła
        self._readings     = 0
        self._sweep_active = False
        self._boat_arrived = False
        self._t_enter      = 0.0
        self._t_start      = 0.0

        # Serwisy klienckie
        self._cli_boat     = self.create_client(SetBoatPose, "/set_boat_pose")
        self._cli_teleport = self.create_client(SetBoatPose, "/teleport_boat")
        self._cli_sweep    = self.create_client(StartSweep,  "/start_sweep")
        self._cli_save     = self.create_client(Empty, "/scan_collector_node/save_csv")

        # Serwisy oferowane
        self.create_service(Trigger, "/mission/start_baseline", self._srv_start_baseline)
        self.create_service(Trigger, "/mission/start_sweep",    self._srv_start_sweep)
        self.create_service(Trigger, "/mission/abort",          self._srv_abort)

        # Topiki
        self.create_subscription(LaserScan, "/sim/sonar",      self._cb_sonar,        10)
        self.create_subscription(Bool,      "/sweep_active",   self._cb_sweep_active, 10)
        self.create_subscription(Bool,      "/boat_arrived",   self._cb_boat_arrived, 10)
        self._pub_status = self.create_publisher(String, "/mission/status", 10)

        # Pętla maszyny stanów — 100 Hz
        self.create_timer(0.01, self._tick)

        self.get_logger().info("MissionSupervisorNode gotowy.")

    # ---------------------------------------------------------------- services

    def _srv_start_baseline(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._start_mission("baseline", response)

    def _srv_start_sweep(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._start_mission("sweep", response)

    def _start_mission(self, scenario: str, response: Trigger.Response) -> Trigger.Response:
        if self._state != State.IDLE:
            response.success = False
            response.message = f"Misja już trwa (stan: {self._state.name})"
            return response

        if scenario == "baseline":
            csv_path = self.get_parameter("waypoints_file").value
        else:
            csv_path = self.get_parameter("sweep_waypoints_file").value

        waypoints = self._load_csv(csv_path)
        if not waypoints:
            response.success = False
            response.message = f"Brak waypointów: {csv_path}"
            return response

        self._scenario     = scenario
        self._waypoints    = waypoints
        self._wp_idx       = 0
        self._wp_sent      = 0
        self._wp_completed = 0

        if scenario == "baseline":
            buf = self.get_parameter("boat_buffer_size").value
            # Teleport do pierwszego waypointu — łódka nie płynie ze spawn point
            wp0 = waypoints[0]
            req = SetBoatPose.Request()
            req.x, req.y = float(wp0[0]), float(wp0[1])
            self._cli_teleport.call_async(req)
            # wp[0] zaliczony przez teleport — zaczynamy od wp[1]
            self._wp_sent      = 1
            self._wp_completed = 1
            self._t_start      = time.monotonic()  # ETA liczy tylko czas pływania
            n = min(buf, len(waypoints) - 1)
            for _ in range(n):
                self._enqueue_next_waypoint()
            self._set_state(State.CRUISING)
            info = (f"Baseline: {len(waypoints)} waypointów, "
                    f"bufor={buf}")
        else:
            self._t_start = time.monotonic()
            self._set_state(State.SEND_GOAL)
            info = (f"Sweep: {len(waypoints)} waypointów, "
                    f"range={self.get_parameter('sweep_range_deg').value}°, "
                    f"step={self.get_parameter('sweep_step_deg').value}°")

        self.get_logger().info(f"Misja startuje — {info}")
        response.success = True
        response.message = info
        return response

    def _srv_abort(self, _req, response: Trigger.Response) -> Trigger.Response:
        if self._state == State.IDLE:
            response.success = False
            response.message = "Brak aktywnej misji."
            return response

        msg = f"Misja przerwana na waypoincie {self._wp_idx}/{len(self._waypoints)}."
        self.get_logger().warn(msg)
        self._set_state(State.IDLE)
        response.success = True
        response.message = msg
        return response

    # ---------------------------------------------------------------- callbacks

    def _cb_sonar(self, _msg: LaserScan) -> None:
        if self._state == State.WAITING_SONAR:
            self._readings += 1

    def _cb_sweep_active(self, msg: Bool) -> None:
        self._sweep_active = msg.data

    def _cb_boat_arrived(self, _msg: Bool) -> None:
        if self._state == State.CRUISING:
            self._wp_completed += 1
            self._log_progress(self._wp_completed)
            if self._wp_sent < len(self._waypoints):
                self._enqueue_next_waypoint()
            if self._wp_completed >= len(self._waypoints):
                self._set_state(State.DONE)
        else:
            self._boat_arrived = True

    # ---------------------------------------------------------------- tick

    def _tick(self) -> None:
        if self._state == State.IDLE:
            return

        if self._state == State.CRUISING:
            pass  # postęp i uzupełnianie kolejki obsługiwane w _cb_boat_arrived

        elif self._state == State.SEND_GOAL:
            self._do_send_goal()

        elif self._state == State.MOVING:
            if self._boat_arrived:
                self._boat_arrived = False
                self._set_state(State.STABILIZE)

        elif self._state == State.STABILIZE:
            stab = self.get_parameter("sweep_stabilize_time").value
            if time.monotonic() - self._t_enter >= stab:
                self._set_state(State.SWEEP_START)

        elif self._state == State.WAITING_SONAR:
            if self._readings >= self.get_parameter("n_readings").value:
                self._set_state(State.NEXT)

        elif self._state == State.SWEEP_START:
            self._do_sweep_start()

        elif self._state == State.SWEEPING:
            if not self._sweep_active:
                self._set_state(State.NEXT)

        elif self._state == State.NEXT:
            self._do_next()

        elif self._state == State.DONE:
            self._do_done()

    # ---------------------------------------------------------------- actions

    def _enqueue_next_waypoint(self) -> None:
        """Wysyła następny waypoint do kolejki nawigacyjnej Isaaca i inkrementuje _wp_sent."""
        wp = self._waypoints[self._wp_sent]
        req = SetBoatPose.Request()
        req.x, req.y = float(wp[0]), float(wp[1])
        self._cli_boat.call_async(req)
        self._wp_sent += 1

    def _log_progress(self, done: int) -> None:
        total    = len(self._waypoints)
        interval = self.get_parameter("progress_interval").value
        if interval > 0:
            should_log = (done % interval == 0 or done == total)
        else:
            prev_pct = int(100.0 * (done - 1) / total / 5) * 5
            curr_pct = int(100.0 * done / total / 5) * 5
            should_log = (curr_pct > prev_pct or done == total)
        if should_log:
            pct = 100.0 * done / total
            eta = self._eta_str(done, total)
            msg = f"[{done:>5}/{total}] {pct:5.1f}%  ETA: {eta}"
            self.get_logger().info(msg)
            self._pub_status.publish(String(data=f"SCANNING {msg}"))

    def _do_send_goal(self) -> None:
        wp = self._waypoints[self._wp_idx]
        req = SetBoatPose.Request()
        req.x, req.y = float(wp[0]), float(wp[1])
        self._boat_arrived = False
        self._cli_boat.call_async(req)
        self._set_state(State.MOVING)

    def _do_sweep_start(self) -> None:
        req = StartSweep.Request()
        req.range_deg     = float(self.get_parameter("sweep_range_deg").value)
        req.step_deg      = float(self.get_parameter("sweep_step_deg").value)
        req.tolerance_deg = float(self.get_parameter("sweep_tolerance_deg").value)
        self._sweep_active = True   # zakładamy True, poczekamy na False
        self._cli_sweep.call_async(req)
        self._set_state(State.SWEEPING)

    def _do_next(self) -> None:
        self._wp_idx += 1
        self._log_progress(self._wp_idx)
        if self._wp_idx >= len(self._waypoints):
            self._set_state(State.DONE)
        else:
            self._set_state(State.SEND_GOAL)

    def _do_done(self) -> None:
        elapsed = time.monotonic() - self._t_start
        self.get_logger().info(
            f"Misja zakończona — {len(self._waypoints)} waypointów "
            f"w {elapsed/60:.1f} min. Zapisuję CSV..."
        )
        self._cli_save.call_async(Empty.Request())
        self._pub_status.publish(String(data="DONE"))
        self._set_state(State.IDLE)

    # ---------------------------------------------------------------- helpers

    def _set_state(self, state: State) -> None:
        self._state   = state
        self._t_enter = time.monotonic()
        # Publikuj tylko istotne zmiany stanu — nie każdy TELEPORT
        if state in (State.IDLE, State.DONE):
            self._pub_status.publish(String(data=state.name))

    def _eta_str(self, done: int, total: int) -> str:
        if done == 0:
            return "?"
        elapsed   = time.monotonic() - self._t_start
        remaining = (elapsed / done) * (total - done)
        if remaining < 60:
            return f"{remaining:.0f}s"
        return f"{remaining/60:.0f} min"

    def _load_csv(self, path: str) -> list[tuple[float, float, float]]:
        if not os.path.isfile(path):
            self.get_logger().error(f"Plik nie istnieje: {path}")
            return []
        waypoints = []
        try:
            with open(path, newline="") as f:
                for row in csv.DictReader(f):
                    waypoints.append((float(row["world_x"]),
                                      float(row["world_y"]),
                                      float(row["world_z"])))
        except Exception as e:
            self.get_logger().error(f"Błąd CSV: {e}")
        self.get_logger().info(f"Wczytano {len(waypoints)} waypointów z {path}")
        return waypoints


def main():
    rclpy.init()
    node = MissionSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
