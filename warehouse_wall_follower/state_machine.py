#!/usr/bin/env python3
"""
state_machine.py
================
ROS2 Node: StateMachineNode

PURPOSE:
  Implements the Finite State Machine (FSM) that orchestrates the
  warehouse inspection robot's high-level behaviour.

  The FSM reads sensor data (wall distance, wall info) and issues
  commands to the PID controller and directly publishes cmd_vel
  for states where PID is not appropriate (SEARCH_WALL, TURN_CORNER).

STATES:
  ┌──────────────┐
  │ SEARCH_WALL  │ Robot spins slowly searching for a wall on its right.
  └──────┬───────┘
         │ wall_detected AND distance < WALL_FOUND_THRESHOLD
         ▼
  ┌──────────────┐
  │ FOLLOW_WALL  │ PID controller active. Robot moves forward maintaining
  └──────┬───────┘ target distance from right wall.
         │         │
         │ distance > FAR_THRESHOLD    distance < CLOSE_THRESHOLD
         │         │                            │
         ▼         ▼                            ▼
  ┌──────────────┐                    ┌──────────────────┐
  │  LOST_WALL   │ wall_detected=False│  FOLLOW_WALL     │
  └──────┬───────┘ for > LOST_TIMEOUT │  (PID corrects)  │
         │                            └──────────────────┘
         │ wall_detected=True
         │ (wall found again within RECOVERY_TIMEOUT)
         ▼
  ┌──────────────┐
  │  TURN_CORNER │ Robot detects front obstacle AND no right wall.
  └──────┬───────┘ Executes timed left turn.
         │ turn complete
         ▼
  ┌──────────────┐
  │ FOLLOW_WALL  │ Resume wall following after corner.
  └──────────────┘

STATE TRANSITION TABLE:
  FROM          TO             CONDITION
  SEARCH_WALL → FOLLOW_WALL:  wall_distance < WALL_FOUND_THRESHOLD (2.0m)
  FOLLOW_WALL → LOST_WALL:    wall_distance = -1.0 for > 1.5 seconds
  FOLLOW_WALL → TURN_CORNER:  front_dist < FRONT_OBSTACLE (0.5m) AND
                               wall_distance > FAR_THRESHOLD (1.5m)
  LOST_WALL   → FOLLOW_WALL:  wall_distance > 0 AND < WALL_FOUND_THRESHOLD
  LOST_WALL   → SEARCH_WALL:  lost duration > RECOVERY_TIMEOUT (8.0s)
  TURN_CORNER → FOLLOW_WALL:  turn duration > TURN_DURATION (2.5s)

TOPICS SUBSCRIBED:
  /wall_distance  (std_msgs/Float64) — filtered wall distance
  /wall_info      (std_msgs/String)  — wall detection diagnostics JSON
  /scan           (sensor_msgs/LaserScan) — for front obstacle detection

TOPICS PUBLISHED:
  /pid_enabled    (std_msgs/Bool)    — enable/disable PID controller
  /cmd_vel        (geometry_msgs/Twist) — direct commands in non-PID states
  /robot_state    (std_msgs/String)  — current FSM state name
  /fsm_diagnostics (std_msgs/String) — detailed FSM status JSON

PARAMETERS:
  wall_found_threshold  (float, 2.00) — max distance to accept as "wall found"
  far_threshold         (float, 1.50) — distance above which wall is "far"
  close_threshold       (float, 0.25) — distance below which wall is "too close"
  front_obstacle_dist   (float, 0.50) — front clearance for corner detection
  lost_timeout          (float, 1.50) — seconds before FOLLOW → LOST transition
  recovery_timeout      (float, 8.00) — seconds before LOST → SEARCH transition
  search_angular_vel    (float, 0.35) — spin speed in SEARCH_WALL state [rad/s]
  turn_angular_vel      (float, 0.60) — turn speed in TURN_CORNER state [rad/s]
  turn_duration         (float, 2.50) — duration of corner turn [s]
  control_frequency     (float, 20.0) — FSM update rate [Hz]
  front_sector_deg      (float, 30.0) — half-width of front obstacle sector
"""

import json
import math
import time
import enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool, String


# ──────────────────────────────────────────────────────────────────────────────
# STATE ENUMERATION
# ──────────────────────────────────────────────────────────────────────────────

class RobotState(enum.Enum):
    """
    All possible FSM states.
    String values are used for logging and /robot_state topic.
    """
    SEARCH_WALL  = 'SEARCH_WALL'   # Spinning to find a wall
    FOLLOW_WALL  = 'FOLLOW_WALL'   # PID active, following wall
    TURN_CORNER  = 'TURN_CORNER'   # Executing timed corner turn
    LOST_WALL    = 'LOST_WALL'     # Wall disappeared; recovery mode


# ──────────────────────────────────────────────────────────────────────────────
# STATE MACHINE NODE
# ──────────────────────────────────────────────────────────────────────────────

class StateMachineNode(Node):
    """
    Finite State Machine controller for warehouse wall-following robot.
    Reads wall distance and scan data, manages state transitions,
    and coordinates with PID controller via /pid_enabled topic.
    """

    def __init__(self):
        super().__init__('state_machine')

        # ── PARAMETER DECLARATIONS ─────────────────────────────────────
        self.declare_parameter('wall_found_threshold',  2.00)
        self.declare_parameter('far_threshold',         1.50)
        self.declare_parameter('close_threshold',       0.25)
        self.declare_parameter('front_obstacle_dist',   0.50)
        self.declare_parameter('lost_timeout',          1.50)
        self.declare_parameter('recovery_timeout',      8.00)
        self.declare_parameter('search_angular_vel',    0.35)
        self.declare_parameter('turn_angular_vel',      0.60)
        self.declare_parameter('turn_duration',         2.50)
        self.declare_parameter('control_frequency',    20.0)
        self.declare_parameter('front_sector_deg',     30.0)

        # ── LOAD PARAMETERS ────────────────────────────────────────────
        self.wall_found_threshold  = self.get_parameter('wall_found_threshold').value
        self.far_threshold         = self.get_parameter('far_threshold').value
        self.close_threshold       = self.get_parameter('close_threshold').value
        self.front_obstacle_dist   = self.get_parameter('front_obstacle_dist').value
        self.lost_timeout          = self.get_parameter('lost_timeout').value
        self.recovery_timeout      = self.get_parameter('recovery_timeout').value
        self.search_angular_vel    = self.get_parameter('search_angular_vel').value
        self.turn_angular_vel      = self.get_parameter('turn_angular_vel').value
        self.turn_duration         = self.get_parameter('turn_duration').value
        control_frequency          = self.get_parameter('control_frequency').value
        front_sector_deg           = self.get_parameter('front_sector_deg').value

        # Front obstacle sector: rays within ±front_sector_deg of forward
        self._front_min_rad = -math.radians(front_sector_deg)
        self._front_max_rad =  math.radians(front_sector_deg)

        # ── FSM STATE ─────────────────────────────────────────────────
        self._state = RobotState.SEARCH_WALL
        self._prev_state = None

        # Timestamps for timeout-based transitions
        self._state_entry_time = self.get_clock().now()
        self._wall_lost_time   = None   # time when wall was last seen

        # ── SENSOR DATA ───────────────────────────────────────────────
        self._wall_distance  = -1.0   # from /wall_distance
        self._front_distance = 99.0   # minimum distance in front sector
        self._wall_detected  = False  # from /wall_info parsed JSON

        # ── QoS PROFILES ───────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── SUBSCRIBERS ────────────────────────────────────────────────
        self.wall_dist_sub = self.create_subscription(
            Float64, '/wall_distance',
            self._wall_distance_callback, reliable_qos
        )

        self.wall_info_sub = self.create_subscription(
            String, '/wall_info',
            self._wall_info_callback, reliable_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self._scan_callback, best_effort_qos
        )

        # ── PUBLISHERS ─────────────────────────────────────────────────
        self.pid_enabled_pub = self.create_publisher(
            Bool, '/pid_enabled', reliable_qos
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', reliable_qos
        )

        self.robot_state_pub = self.create_publisher(
            String, '/robot_state', reliable_qos
        )

        self.fsm_diag_pub = self.create_publisher(
            String, '/fsm_diagnostics', reliable_qos
        )

        # ── FSM TIMER ─────────────────────────────────────────────────
        timer_period = 1.0 / control_frequency
        self.fsm_timer = self.create_timer(
            timer_period,
            self._fsm_update_callback
        )

        self._update_count = 0

        self.get_logger().info('StateMachineNode initialised. Initial state: SEARCH_WALL')
        self._publish_pid_enabled(False)  # PID off until wall found

    # ──────────────────────────────────────────────────────────────────
    # SUBSCRIBER CALLBACKS
    # ──────────────────────────────────────────────────────────────────

    def _wall_distance_callback(self, msg: Float64) -> None:
        """Store latest wall distance. -1.0 = not detected."""
        self._wall_distance = msg.data

    def _wall_info_callback(self, msg: String) -> None:
        """
        Parse wall_info JSON to extract detected status.
        JSON format: {"distance": 0.72, "detected": true, ...}
        """
        try:
            info = json.loads(msg.data)
            self._wall_detected = info.get('detected', False)
        except (json.JSONDecodeError, KeyError):
            self._wall_detected = False

    def _scan_callback(self, scan_msg: LaserScan) -> None:
        """
        Extract minimum distance in front sector for obstacle detection.
        Front sector: rays within ±front_sector_deg of forward (angle=0).

        The front distance is used by FOLLOW_WALL to detect corners:
        if something is directly ahead AND the right wall is missing/far,
        the robot initiates a corner turn.
        """
        front_readings = []

        for i, range_val in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Only look in the forward-facing sector
            if not (self._front_min_rad <= angle <= self._front_max_rad):
                continue

            if not math.isfinite(range_val):
                continue
            if range_val < scan_msg.range_min:
                continue
            if range_val > scan_msg.range_max:
                continue

            front_readings.append(range_val)

        if front_readings:
            # Minimum distance in front sector = closest obstacle ahead
            self._front_distance = min(front_readings)
        else:
            # No valid readings = assume clear path (sensor issue / open space)
            self._front_distance = 99.0

    # ──────────────────────────────────────────────────────────────────
    # MAIN FSM UPDATE (20 Hz timer callback)
    # ──────────────────────────────────────────────────────────────────

    def _fsm_update_callback(self) -> None:
        """
        Called at 20 Hz. Evaluates current state, checks transition
        conditions, executes state actions, and handles transitions.

        Structure:
          For each state:
            1. Execute state action (issue velocity commands)
            2. Check transition conditions
            3. If transition: call _transition_to(new_state)
        """
        self._update_count += 1
        now = self.get_clock().now()

        # ── STATE: SEARCH_WALL ─────────────────────────────────────────
        if self._state == RobotState.SEARCH_WALL:
            self._execute_search_wall()

            # TRANSITION: wall detected within acceptable range
            if (self._wall_detected and
                    0.0 < self._wall_distance < self.wall_found_threshold):
                self.get_logger().info(
                    f'SEARCH_WALL → FOLLOW_WALL: '
                    f'wall found at {self._wall_distance:.3f}m'
                )
                self._transition_to(RobotState.FOLLOW_WALL)

        # ── STATE: FOLLOW_WALL ─────────────────────────────────────────
        elif self._state == RobotState.FOLLOW_WALL:
            # PID controller handles velocity in this state.
            # FSM only monitors for transition conditions.
            self._execute_follow_wall()

            # TRANSITION 1: Wall disappeared → LOST_WALL
            if not self._wall_detected or self._wall_distance < 0.0:
                if self._wall_lost_time is None:
                    # Record when we first lost the wall
                    self._wall_lost_time = now
                    self.get_logger().warning(
                        'Wall signal lost. Starting lost_timeout countdown...'
                    )
                else:
                    # Check if we've been lost for too long
                    lost_duration = (now - self._wall_lost_time).nanoseconds * 1e-9
                    if lost_duration > self.lost_timeout:
                        self.get_logger().warning(
                            f'FOLLOW_WALL → LOST_WALL: '
                            f'wall missing for {lost_duration:.2f}s'
                        )
                        self._transition_to(RobotState.LOST_WALL)
            else:
                # Wall present; reset lost timer
                self._wall_lost_time = None

            # TRANSITION 2: Front obstacle AND wall far → TURN_CORNER
            if (self._front_distance < self.front_obstacle_dist and
                    (not self._wall_detected or
                     self._wall_distance > self.far_threshold)):
                self.get_logger().info(
                    f'FOLLOW_WALL → TURN_CORNER: '
                    f'front={self._front_distance:.2f}m, '
                    f'wall={self._wall_distance:.2f}m'
                )
                self._transition_to(RobotState.TURN_CORNER)

        # ── STATE: TURN_CORNER ─────────────────────────────────────────
        elif self._state == RobotState.TURN_CORNER:
            self._execute_turn_corner()

            # TRANSITION: timed turn complete
            elapsed = (now - self._state_entry_time).nanoseconds * 1e-9
            if elapsed >= self.turn_duration:
                self.get_logger().info(
                    f'TURN_CORNER → FOLLOW_WALL: '
                    f'turn complete after {elapsed:.2f}s'
                )
                self._transition_to(RobotState.FOLLOW_WALL)

        # ── STATE: LOST_WALL ───────────────────────────────────────────
        elif self._state == RobotState.LOST_WALL:
            self._execute_lost_wall()

            # TRANSITION 1: Wall found again → FOLLOW_WALL
            if (self._wall_detected and
                    0.0 < self._wall_distance < self.wall_found_threshold):
                self.get_logger().info(
                    f'LOST_WALL → FOLLOW_WALL: '
                    f'wall reacquired at {self._wall_distance:.3f}m'
                )
                self._transition_to(RobotState.FOLLOW_WALL)

            # TRANSITION 2: Recovery timeout → SEARCH_WALL
            else:
                elapsed = (now - self._state_entry_time).nanoseconds * 1e-9
                if elapsed > self.recovery_timeout:
                    self.get_logger().warning(
                        f'LOST_WALL → SEARCH_WALL: '
                        f'recovery timeout after {elapsed:.2f}s'
                    )
                    self._transition_to(RobotState.SEARCH_WALL)

        # ── PUBLISH STATE AND DIAGNOSTICS ─────────────────────────────
        self._publish_state()

        if self._update_count % 20 == 0:
            self._publish_diagnostics()

    # ──────────────────────────────────────────────────────────────────
    # STATE EXECUTION METHODS
    # Each method contains the velocity commands for that state.
    # ──────────────────────────────────────────────────────────────────

    def _execute_search_wall(self) -> None:
        """
        SEARCH_WALL execution:
        Robot rotates counter-clockwise (angular.z > 0) at
        search_angular_vel rad/s while keeping linear velocity zero.
        This causes the robot to spin in place until its right side
        faces a wall within wall_found_threshold distance.

        We rotate LEFT (CCW) so the robot's right side sweeps the
        environment. When the right-side sensor detects a wall,
        the robot is oriented to begin following.
        """
        twist = Twist()
        twist.linear.x  = 0.0
        twist.angular.z = self.search_angular_vel  # counter-clockwise spin
        self.cmd_vel_pub.publish(twist)

    def _execute_follow_wall(self) -> None:
        """
        FOLLOW_WALL execution:
        The PID controller handles velocity commands via its own
        20 Hz timer. The FSM does NOT publish cmd_vel in this state.
        The FSM only enables/disables the PID and monitors transitions.

        The PID enable is set during _transition_to(FOLLOW_WALL).
        No velocity command is needed here.
        """
        pass  # PID controller is active; FSM monitors only

    def _execute_turn_corner(self) -> None:
        """
        TURN_CORNER execution:
        Robot slows forward motion and turns LEFT (counter-clockwise)
        to navigate around a corner.

        Turn strategy:
          - Reduce forward speed to 0.08 m/s (slow crawl through corner)
          - Apply strong left turn at turn_angular_vel rad/s
          - This combination traces a left arc of radius ≈ 0.13m
          - Turn duration set by turn_duration parameter (2.5s default)

        After the turn, FOLLOW_WALL resumes and the robot should find
        the new wall on its right side.
        """
        twist = Twist()
        twist.linear.x  = 0.08                    # slow forward during turn
        twist.angular.z = self.turn_angular_vel    # counter-clockwise turn
        self.cmd_vel_pub.publish(twist)

    def _execute_lost_wall(self) -> None:
        """
        LOST_WALL execution:
        Robot moves forward slowly while the FSM waits for wall reacquisition.

        Rationale: if the robot lost the wall due to a gap in storage racks
        (common in warehouses), continuing forward often brings the robot
        to the next rack, re-establishing the wall signal.

        If no wall is found within recovery_timeout, transition to SEARCH_WALL.

        Velocity: 0.10 m/s forward, no rotation (straight ahead).
        """
        twist = Twist()
        twist.linear.x  = 0.10   # slow creep forward
        twist.angular.z = 0.0    # no rotation; let PID reactivate when wall found
        self.cmd_vel_pub.publish(twist)

    # ──────────────────────────────────────────────────────────────────
    # STATE TRANSITION HANDLER
    # ──────────────────────────────────────────────────────────────────

    def _transition_to(self, new_state: RobotState) -> None:
        """
        Perform a state transition.

        Actions taken on transition:
          1. Record previous state for logging.
          2. Update state entry time (used by timeout transitions).
          3. On entering FOLLOW_WALL: enable PID controller.
          4. On leaving  FOLLOW_WALL: disable PID controller.
          5. On entering TURN_CORNER or LOST_WALL or SEARCH_WALL:
             ensure PID is disabled (FSM takes direct control).
          6. Reset wall_lost_time tracker.
          7. Log the transition.

        Args:
            new_state: The RobotState to transition to.
        """
        self._prev_state     = self._state
        self._state          = new_state
        self._state_entry_time = self.get_clock().now()
        self._wall_lost_time   = None  # Reset on any transition

        # Determine PID enable state for the new state
        if new_state == RobotState.FOLLOW_WALL:
            # Enable PID: it will handle velocity commands
            self._publish_pid_enabled(True)
            self.get_logger().info(f'→ {new_state.value}: PID ENABLED')

        elif new_state in (RobotState.SEARCH_WALL,
                           RobotState.TURN_CORNER,
                           RobotState.LOST_WALL):
            # Disable PID: FSM takes direct velocity control
            self._publish_pid_enabled(False)
            self.get_logger().info(f'→ {new_state.value}: PID DISABLED')

    # ──────────────────────────────────────────────────────────────────
    # PUBLISHER HELPERS
    # ──────────────────────────────────────────────────────────────────

    def _publish_pid_enabled(self, enabled: bool) -> None:
        """Publish PID enable/disable command."""
        msg = Bool()
        msg.data = enabled
        self.pid_enabled_pub.publish(msg)

    def _publish_state(self) -> None:
        """Publish current FSM state as a String message."""
        msg = String()
        msg.data = self._state.value
        self.robot_state_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        """
        Publish detailed FSM diagnostics as JSON string.
        Useful for debugging state transitions and sensor status.
        """
        now = self.get_clock().now()
        elapsed = (now - self._state_entry_time).nanoseconds * 1e-9

        diag = {
            'state':          self._state.value,
            'prev_state':     self._prev_state.value if self._prev_state else 'NONE',
            'wall_distance':  round(self._wall_distance, 4),
            'wall_detected':  self._wall_detected,
            'front_distance': round(self._front_distance, 4),
            'state_elapsed_s': round(elapsed, 2),
        }
        msg = String()
        msg.data = json.dumps(diag)
        self.fsm_diag_pub.publish(msg)

        self.get_logger().info(
            f'FSM [{self._state.value}] | '
            f'wall={self._wall_distance:.3f}m | '
            f'front={self._front_distance:.3f}m | '
            f'elapsed={elapsed:.1f}s'
        )


def main(args=None):
    """Entry point for ros2 run warehouse_wall_follower state_machine."""
    rclpy.init(args=args)

    node = StateMachineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('StateMachineNode shutting down.')
        # Stop robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node._publish_pid_enabled(False)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
