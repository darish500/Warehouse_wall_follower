#!/usr/bin/env python3
"""
metrics_node.py
===============
ROS2 Node: MetricsNode

PURPOSE:
  Subscribes to odometry and wall distance topics, computes real-time
  performance metrics, and publishes them as a JSON diagnostics string.
  Also logs a formatted summary to the ROS2 console every 10 seconds.

METRICS COMPUTED:
  avg_wall_error:
    Running mean of |wall_distance - target_distance| in metres.
    Lower = better tracking accuracy.
    Formula: avg = sum(|e_i|) / n

  max_wall_deviation:
    Maximum absolute error ever recorded during the run.
    Identifies worst-case PID performance.

  tracking_accuracy_pct:
    Percentage of samples where |error| < ACCURACY_THRESHOLD (0.15m).
    Formula: 100 × (n_accurate / n_total)
    A reading within 15cm of target is considered "on track".

  total_distance_m:
    Total path length travelled by the robot, computed from odometry.
    Uses Euclidean distance between consecutive pose samples:
      Δd = sqrt((x_new - x_old)² + (y_new - y_old)²)
      total += Δd

  wall_detected_pct:
    Percentage of time the wall was successfully detected.
    Formula: 100 × (n_detected / n_total)

  run_duration_s:
    Total elapsed time since node started.

SUBSCRIBED TOPICS:
  /wall_distance  (std_msgs/Float64)  — from wall_detector
  /odom           (nav_msgs/Odometry) — from DiffDrive plugin / ros_gz_bridge

PUBLISHED TOPICS:
  /metrics        (std_msgs/String)   — JSON metrics string, published at 1 Hz

PARAMETERS:
  target_distance     (float, 0.70)   — target wall distance [m]
  accuracy_threshold  (float, 0.15)   — max error for "on-track" classification
  publish_rate        (float, 1.0)    — metrics publication rate [Hz]
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String


class MetricsNode(Node):
    """
    Computes and publishes performance metrics for wall-following robot.
    """

    def __init__(self):
        super().__init__('metrics_node')

        # ── PARAMETERS ────────────────────────────────────────────────
        self.declare_parameter('target_distance',    0.70)
        self.declare_parameter('accuracy_threshold', 0.15)
        self.declare_parameter('publish_rate',       1.0)

        self.target_distance    = self.get_parameter('target_distance').value
        self.accuracy_threshold = self.get_parameter('accuracy_threshold').value
        publish_rate            = self.get_parameter('publish_rate').value

        # ── METRIC ACCUMULATORS ───────────────────────────────────────
        self._total_samples      = 0    # total wall_distance readings
        self._detected_samples   = 0    # samples where wall was detected
        self._accurate_samples   = 0    # samples within accuracy_threshold
        self._sum_abs_error      = 0.0  # sum of |error| for mean calc
        self._max_deviation      = 0.0  # maximum |error| seen

        # ── ODOMETRY TRACKING ─────────────────────────────────────────
        self._prev_x         = None   # previous odometry X position
        self._prev_y         = None   # previous odometry Y position
        self._total_distance = 0.0    # accumulated path length [m]

        # ── TIME TRACKING ─────────────────────────────────────────────
        self._start_time = self.get_clock().now()

        # ── QoS PROFILES ─────────────────────────────────────────────
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

        # ── SUBSCRIBERS ───────────────────────────────────────────────
        self.wall_dist_sub = self.create_subscription(
            Float64, '/wall_distance',
            self._wall_distance_callback, reliable_qos
        )

        # Odometry: BEST_EFFORT since ros_gz_bridge publishes it that way
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self._odom_callback, best_effort_qos
        )

        # ── PUBLISHERS ────────────────────────────────────────────────
        self.metrics_pub = self.create_publisher(
            String, '/metrics', reliable_qos
        )

        # ── PUBLISH TIMER ─────────────────────────────────────────────
        timer_period = 1.0 / publish_rate
        self.metrics_timer = self.create_timer(
            timer_period,
            self._publish_metrics
        )

        self.get_logger().info('MetricsNode initialised. Recording performance data.')

    # ──────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ──────────────────────────────────────────────────────────────────

    def _wall_distance_callback(self, msg: Float64) -> None:
        """
        Process wall distance reading and update metric accumulators.

        Increments:
          _total_samples      always
          _detected_samples   if wall was detected (distance > 0)
          sum_abs_error       with |distance - target| if detected
          _max_deviation      if current error exceeds previous max
          _accurate_samples   if error < accuracy_threshold
        """
        self._total_samples += 1
        distance = msg.data

        if distance < 0.0:
            # Wall not detected; count as miss but don't add to error stats
            return

        self._detected_samples += 1

        # Absolute error from target
        abs_error = abs(distance - self.target_distance)

        # Accumulate for mean calculation
        self._sum_abs_error += abs_error

        # Update maximum deviation
        if abs_error > self._max_deviation:
            self._max_deviation = abs_error

        # Count accurate samples (within threshold)
        if abs_error <= self.accuracy_threshold:
            self._accurate_samples += 1

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Process odometry to compute total distance travelled.

        Extracts robot X,Y position from the pose and computes
        Euclidean distance from the previous known position.
        Accumulates into _total_distance.

        Uses nav_msgs/Odometry::pose::pose::position.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._prev_x is None:
            # First reading: initialise without adding distance
            self._prev_x = x
            self._prev_y = y
            return

        # Euclidean distance between consecutive positions
        dx = x - self._prev_x
        dy = y - self._prev_y
        step_distance = math.sqrt(dx * dx + dy * dy)

        # Filter very small steps (odometry noise) and large jumps (teleport)
        if 0.001 <= step_distance <= 0.5:
            self._total_distance += step_distance

        self._prev_x = x
        self._prev_y = y

    # ──────────────────────────────────────────────────────────────────
    # METRICS COMPUTATION AND PUBLICATION (1 Hz)
    # ──────────────────────────────────────────────────────────────────

    def _publish_metrics(self) -> None:
        """
        Compute derived metrics from accumulators and publish.
        Called at 1 Hz by timer.
        """
        now = self.get_clock().now()
        run_duration = (now - self._start_time).nanoseconds * 1e-9

        # Average wall error (over detected samples only)
        if self._detected_samples > 0:
            avg_error = self._sum_abs_error / self._detected_samples
        else:
            avg_error = 0.0

        # Tracking accuracy percentage (over all samples)
        if self._total_samples > 0:
            tracking_accuracy = 100.0 * self._accurate_samples / self._total_samples
            wall_detected_pct = 100.0 * self._detected_samples / self._total_samples
        else:
            tracking_accuracy = 0.0
            wall_detected_pct = 0.0

        metrics = {
            'run_duration_s':      round(run_duration, 1),
            'total_distance_m':    round(self._total_distance, 3),
            'avg_wall_error_m':    round(avg_error, 4),
            'max_wall_deviation_m': round(self._max_deviation, 4),
            'tracking_accuracy_pct': round(tracking_accuracy, 2),
            'wall_detected_pct':   round(wall_detected_pct, 2),
            'total_samples':       self._total_samples,
            'detected_samples':    self._detected_samples,
            'accurate_samples':    self._accurate_samples,
        }

        msg = String()
        msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)

        # Console summary every 10 seconds
        if int(run_duration) % 10 == 0 and self._total_samples > 0:
            self.get_logger().info(
                f'[METRICS] t={run_duration:.0f}s | '
                f'dist={self._total_distance:.2f}m | '
                f'avg_err={avg_error:.4f}m | '
                f'max_dev={self._max_deviation:.4f}m | '
                f'accuracy={tracking_accuracy:.1f}% | '
                f'wall_up={wall_detected_pct:.1f}%'
            )


def main(args=None):
    """Entry point for ros2 run warehouse_wall_follower metrics_node."""
    rclpy.init(args=args)

    node = MetricsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MetricsNode shutting down. Final metrics:')
        # Print final metrics on shutdown
        if node._total_samples > 0:
            node.get_logger().info(
                f'  Total distance: {node._total_distance:.3f}m\n'
                f'  Total samples: {node._total_samples}\n'
                f'  Max deviation: {node._max_deviation:.4f}m'
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
