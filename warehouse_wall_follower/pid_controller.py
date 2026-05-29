#!/usr/bin/env python3
"""
pid_controller.py
=================
ROS2 Node: PIDControllerNode

PURPOSE:
  Implements a full PID (Proportional-Integral-Derivative) controller
  that maintains the robot at a target distance from the right wall.

  Input:  /wall_distance  (std_msgs/Float64) — current wall distance
  Input:  /pid_enabled    (std_msgs/Bool)    — enable/disable from FSM
  Output: /cmd_vel        (geometry_msgs/Twist) — velocity command

CONTROL LAW:
  The robot needs to maintain TARGET_DISTANCE from the right wall.

  Error definition:
    e(t) = target_distance - measured_distance

  Positive error → robot is TOO FAR from wall → turn right (steer towards wall)
  Negative error → robot is TOO CLOSE to wall → turn left (steer away from wall)

  PID output (angular velocity):
    u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt

  The angular velocity u(t) is then negated because:
    - Positive u(t) = need to turn towards wall (rightward)
    - Rightward turn in ROS2 = negative angular.z
    So: cmd_vel.angular.z = -u(t)  [see code below]

  Forward velocity is constant (set in parameters) when PID is active,
  scaled down during large corrections to maintain stability.

PID ANTI-WINDUP:
  Integral windup occurs when the integrator accumulates large values
  during prolonged error (e.g. robot lost the wall and is spinning).
  Anti-windup strategy: clamp the integral term to [-INTEGRAL_MAX, +INTEGRAL_MAX].
  This is a simple saturation anti-windup (clamping).

SATURATION LIMITS:
  cmd_vel.linear.x  saturated to [0.0, MAX_LINEAR_VEL]
  cmd_vel.angular.z saturated to [-MAX_ANGULAR_VEL, +MAX_ANGULAR_VEL]

PID DIAGNOSTIC TOPIC:
  /pid_diagnostics (std_msgs/String)
  JSON string with: error, p_term, i_term, d_term, output, dt

CONTROL LOOP TIMING:
  Frequency: 20 Hz (period = 0.05s)
  Timer-based: uses rclpy timer for deterministic execution.
  dt is measured from actual wall clock to handle jitter correctly.

PARAMETERS (all tunable via YAML config):
  target_distance   (float, default: 0.70)  — desired wall distance [m]
  kp                (float, default: 1.20)  — proportional gain
  ki                (float, default: 0.05)  — integral gain
  kd                (float, default: 0.80)  — derivative gain
  max_linear_vel    (float, default: 0.25)  — max forward speed [m/s]
  max_angular_vel   (float, default: 1.20)  — max angular speed [rad/s]
  integral_max      (float, default: 1.50)  — anti-windup clamp [rad/s]
  control_frequency (float, default: 20.0)  — control loop rate [Hz]
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String


class PIDControllerNode(Node):
    """
    PID controller maintaining target distance from right wall.
    Outputs geometry_msgs/Twist on /cmd_vel.
    """

    def __init__(self):
        super().__init__('pid_controller')

        # ── PARAMETER DECLARATIONS ─────────────────────────────────────
        self.declare_parameter('target_distance',   0.70)
        self.declare_parameter('kp',                1.20)
        self.declare_parameter('ki',                0.05)
        self.declare_parameter('kd',                0.80)
        self.declare_parameter('max_linear_vel',    0.25)
        self.declare_parameter('max_angular_vel',   1.20)
        self.declare_parameter('integral_max',      1.50)
        self.declare_parameter('control_frequency', 20.0)

        # ── LOAD PARAMETERS ────────────────────────────────────────────
        self.target_distance   = self.get_parameter('target_distance').value
        self.kp                = self.get_parameter('kp').value
        self.ki                = self.get_parameter('ki').value
        self.kd                = self.get_parameter('kd').value
        self.max_linear_vel    = self.get_parameter('max_linear_vel').value
        self.max_angular_vel   = self.get_parameter('max_angular_vel').value
        self.integral_max      = self.get_parameter('integral_max').value
        control_frequency      = self.get_parameter('control_frequency').value

        self.get_logger().info(
            f'PID params: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, '
            f'target={self.target_distance}m, freq={control_frequency}Hz'
        )

        # ── PID STATE VARIABLES ────────────────────────────────────────
        # These accumulate across control loop iterations.

        # integral_term: running sum of (error × dt) — reset on enable
        self._integral = 0.0

        # prev_error: error from previous iteration, for derivative calc
        self._prev_error = 0.0

        # prev_time: ROS time of previous control iteration
        self._prev_time = None

        # ── SENSOR / COMMAND STATE ─────────────────────────────────────
        # Latest wall distance from WallDetectorNode
        self._wall_distance = -1.0   # -1.0 = not detected

        # Whether PID is active (set by FSM via /pid_enabled)
        self._pid_enabled = False

        # ── QoS PROFILES ───────────────────────────────────────────────
        default_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── SUBSCRIBERS ────────────────────────────────────────────────
        # Wall distance from detector node
        self.wall_dist_sub = self.create_subscription(
            Float64,
            '/wall_distance',
            self._wall_distance_callback,
            default_qos
        )

        # Enable/disable from state machine
        self.enabled_sub = self.create_subscription(
            Bool,
            '/pid_enabled',
            self._enabled_callback,
            default_qos
        )

        # ── PUBLISHERS ─────────────────────────────────────────────────
        # Main velocity command — to Gazebo via ros_gz_bridge
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            default_qos
        )

        # PID diagnostics — JSON string for monitoring
        self.diag_pub = self.create_publisher(
            String,
            '/pid_diagnostics',
            default_qos
        )

        # ── CONTROL LOOP TIMER ──────────────────────────────────────────
        # 20 Hz timer: fires control_loop_callback every 0.05 seconds.
        # rclpy timer is wall-clock based; we measure actual dt inside
        # the callback to handle scheduling jitter correctly.
        timer_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(
            timer_period,
            self._control_loop_callback
        )

        # ── DIAGNOSTICS COUNTER ────────────────────────────────────────
        self._loop_count = 0

        self.get_logger().info('PIDControllerNode initialised. Waiting for /pid_enabled=true.')

    # ──────────────────────────────────────────────────────────────────
    # SUBSCRIBER CALLBACKS
    # ──────────────────────────────────────────────────────────────────

    def _wall_distance_callback(self, msg: Float64) -> None:
        """
        Store the latest wall distance reading.
        Called at ~10 Hz (LiDAR update rate).
        Value -1.0 indicates wall not detected.
        """
        self._wall_distance = msg.data

    def _enabled_callback(self, msg: Bool) -> None:
        """
        Enable or disable the PID controller.
        When transitioning to ENABLED: reset PID state to prevent
        integral and derivative transients from previous disable period.
        When transitioning to DISABLED: publish zero velocity to stop robot.
        """
        if msg.data and not self._pid_enabled:
            # Transition: disabled → enabled
            self._reset_pid_state()
            self.get_logger().info('PID Controller ENABLED. Resetting PID state.')

        elif not msg.data and self._pid_enabled:
            # Transition: enabled → disabled
            self._publish_zero_velocity()
            self.get_logger().info('PID Controller DISABLED. Robot stopped.')

        self._pid_enabled = msg.data

    # ──────────────────────────────────────────────────────────────────
    # CONTROL LOOP (20 Hz timer callback)
    # ──────────────────────────────────────────────────────────────────

    def _control_loop_callback(self) -> None:
        """
        Main PID control loop executed at 20 Hz.

        STEPS:
          1. Guard: return if PID disabled or wall not detected.
          2. Compute dt (time since last iteration).
          3. Compute error: e = target_distance - wall_distance.
          4. Compute P term: Kp × e
          5. Compute I term: accumulate with anti-windup clamping.
          6. Compute D term: Kd × de/dt  (derivative of error)
          7. Sum P+I+D → PID output u.
          8. Map PID output to angular velocity (with sign flip).
          9. Scale forward velocity based on correction magnitude.
          10. Saturate both velocities.
          11. Publish Twist.
          12. Update state for next iteration.
          13. Publish diagnostics.
        """
        self._loop_count += 1

        # ── STEP 1: Guard conditions ───────────────────────────────────
        if not self._pid_enabled:
            return  # FSM has disabled PID; do nothing

        if self._wall_distance < 0.0:
            # Wall not detected; don't command motion (state machine handles this)
            return

        # ── STEP 2: Compute dt ────────────────────────────────────────
        now = self.get_clock().now()

        if self._prev_time is None:
            # First iteration: no derivative yet; initialise timekeeping
            self._prev_time = now
            self._prev_error = self.target_distance - self._wall_distance
            return  # Skip this iteration; wait for second tick

        # dt = elapsed seconds since last control iteration
        # rclpy.Time subtraction returns rclpy.Duration; convert to float
        dt = (now - self._prev_time).nanoseconds * 1e-9  # seconds

        # Guard against zero or negative dt (clock rollback / scheduler glitch)
        if dt <= 0.0:
            self.get_logger().warning('dt <= 0 in PID loop; skipping iteration.')
            return

        # Cap dt to prevent huge derivative spikes after pauses
        # (e.g. if node was starved by OS scheduler for >0.5s)
        dt = min(dt, 0.1)  # max 100ms; larger gaps treated as 100ms

        # ── STEP 3: Compute error ─────────────────────────────────────
        # Positive error → robot is farther from wall than desired → steer right
        # Negative error → robot is closer to wall than desired  → steer left
        error = self.target_distance - self._wall_distance

        # ── STEP 4: Proportional term ─────────────────────────────────
        # P responds to the CURRENT error magnitude.
        # Larger Kp → faster response, but risks oscillation.
        p_term = self.kp * error

        # ── STEP 5: Integral term with anti-windup ─────────────────────
        # I responds to ACCUMULATED error over time.
        # Eliminates steady-state offset that P alone cannot remove.
        #
        # Accumulation:
        #   integral += error × dt   (Euler integration / rectangle rule)
        #
        # Anti-windup: if integral exceeds ±integral_max, clamp it.
        # This prevents the integrator from winding up large values
        # when the wall is lost (error = constant for many seconds).
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))

        i_term = self.ki * self._integral

        # ── STEP 6: Derivative term ───────────────────────────────────
        # D responds to the RATE OF CHANGE of error.
        # Damps oscillations; "predicts" the error trend.
        #
        # Numerical derivative: (error - prev_error) / dt
        # This is a backwards-difference approximation of de/dt.
        d_term = self.kd * (error - self._prev_error) / dt

        # ── STEP 7: PID sum ───────────────────────────────────────────
        # Total PID output before saturation
        pid_output = p_term + i_term + d_term

        # ── STEP 8: Map PID to angular velocity ───────────────────────
        # Convention:
        #   ROS2 Twist angular.z > 0 → counter-clockwise (turn LEFT)
        #   ROS2 Twist angular.z < 0 → clockwise (turn RIGHT)
        #
        # If error > 0 (too far from wall) → need to turn RIGHT (−angular.z)
        # If error < 0 (too close to wall) → need to turn LEFT  (+angular.z)
        #
        # Therefore: angular.z = -pid_output
        angular_z = -pid_output

        # ── STEP 9: Scale forward velocity based on correction ─────────
        # When angular correction is large, the robot is making a significant
        # turn. Reduce forward speed proportionally to maintain smooth path.
        #
        # correction_ratio = |angular_z| / max_angular_vel  ∈ [0, 1]
        # linear_x = max_linear_vel × (1 - 0.5 × correction_ratio)
        #
        # This ensures:
        #   - Straight path: full forward speed
        #   - Large correction: 50% forward speed minimum
        correction_ratio = min(abs(angular_z) / self.max_angular_vel, 1.0)
        linear_x = self.max_linear_vel * (1.0 - 0.5 * correction_ratio)

        # ── STEP 10: Saturate velocities ─────────────────────────────
        linear_x  = max(0.0, min(self.max_linear_vel, linear_x))
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))

        # ── STEP 11: Publish Twist ────────────────────────────────────
        twist = Twist()
        twist.linear.x  = linear_x
        twist.linear.y  = 0.0
        twist.linear.z  = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

        # ── STEP 12: Update state for next iteration ──────────────────
        self._prev_error = error
        self._prev_time  = now

        # ── STEP 13: Publish diagnostics (every 20 iterations = 1 sec) ─
        if self._loop_count % 20 == 0:
            diag = {
                'error':      round(error, 4),
                'p_term':     round(p_term, 4),
                'i_term':     round(i_term, 4),
                'd_term':     round(d_term, 4),
                'pid_output': round(pid_output, 4),
                'angular_z':  round(angular_z, 4),
                'linear_x':   round(linear_x, 4),
                'dt_ms':      round(dt * 1000.0, 2),
                'integral':   round(self._integral, 4),
            }
            diag_msg = String()
            diag_msg.data = json.dumps(diag)
            self.diag_pub.publish(diag_msg)

            self.get_logger().debug(
                f'PID | e={error:.3f}m | '
                f'P={p_term:.3f} I={i_term:.3f} D={d_term:.3f} | '
                f'cmd: lin={linear_x:.3f} ang={angular_z:.3f}'
            )

    # ──────────────────────────────────────────────────────────────────
    # HELPER METHODS
    # ──────────────────────────────────────────────────────────────────

    def _reset_pid_state(self) -> None:
        """
        Reset integral accumulator, previous error, and time reference.
        Called when PID is (re)enabled to prevent transient spikes from
        stale state accumulated during disabled period.
        """
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def _publish_zero_velocity(self) -> None:
        """
        Publish a zero Twist to immediately halt the robot.
        Called when PID is disabled by the state machine.
        """
        twist = Twist()
        # All fields default to 0.0 in ROS2 message construction
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    """Entry point for ros2 run warehouse_wall_follower pid_controller."""
    rclpy.init(args=args)

    node = PIDControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PIDControllerNode shutting down.')
        # Ensure robot stops on shutdown
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
