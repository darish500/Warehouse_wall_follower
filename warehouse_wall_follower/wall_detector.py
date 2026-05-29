#!/usr/bin/env python3
"""
wall_detector.py
================
ROS2 Node: WallDetectorNode

PURPOSE:
  Subscribes to the /scan topic (sensor_msgs/LaserScan) published
  by the LiDAR sensor and extracts a reliable distance measurement
  to the RIGHT-side wall. Publishes the filtered distance and
  detection status for consumption by the PID controller and
  state machine nodes.

ARCHITECTURE:
  ┌─────────┐    /scan     ┌──────────────────┐   /wall_distance  ┌─────────────┐
  │  LiDAR  │ ──────────► │  WallDetector    │ ────────────────► │  PID Ctrl   │
  │ (Gazebo)│             │  - sector slice  │                   │  State Mach │
  └─────────┘             │  - median filter │   /wall_info      └─────────────┘
                          │  - gap detection │ ────────────────► (state machine)
                          └──────────────────┘

WALL DETECTION STRATEGY:
  The robot follows the RIGHT wall. To detect it we look at LiDAR
  rays pointing to the RIGHT of the robot's forward direction.

  LiDAR coordinate convention (ROS2 REP-103):
    - Angle = 0      → forward (+X robot frame)
    - Angle = +π/2   → LEFT  (+Y robot frame)
    - Angle = -π/2   → RIGHT (-Y robot frame)
    - Angle = ±π     → backward (-X robot frame)

  We sample a 60-degree sector centred at -90° (right side):
    sector_centre = -π/2
    half_width    = π/6  (30° each side)
    active range  = [-π/2 - π/6, -π/2 + π/6]
                  = [-2π/3, -π/3]
                  = [-120°, -60°]

  Within this sector we collect all valid range readings and apply:
    1. Range validity filter: discard NaN, inf, and readings outside
       [min_range, max_range] from the LaserScan message.
    2. Outlier filter: discard readings > OUTLIER_THRESHOLD (3.0m)
       which likely indicate a gap or open space rather than wall.
    3. Median filter: take the median of remaining valid readings.
       Median is chosen over mean because it is robust to single
       erroneous spike readings (e.g. glass or specular surfaces).
    4. Gap detection: if fewer than MIN_VALID_READINGS valid samples
       remain after filtering, the wall is considered absent.

PUBLISHED TOPICS:
  /wall_distance  (std_msgs/Float64)
    The filtered perpendicular distance to the right wall in metres.
    Value = -1.0 signals wall not detected.

  /wall_info      (std_msgs/String)
    JSON-like diagnostic string for state machine and debugging:
    {"distance": 0.72, "detected": true, "valid_samples": 28,
     "sector_min": 0.68, "sector_max": 0.77}

SUBSCRIBED TOPICS:
  /scan  (sensor_msgs/LaserScan)
    Full 360° LiDAR scan from the robot.

PARAMETERS:
  sector_centre_deg   (float, default: -90.0)
    Centre of the wall-detection sector in degrees.
  sector_half_deg     (float, default: 30.0)
    Half-width of the detection sector in degrees.
  outlier_threshold   (float, default: 3.0)
    Maximum valid wall distance in metres (above = gap).
  min_valid_readings  (int, default: 5)
    Minimum samples required to declare wall detected.
  publish_rate        (float, default: 20.0)
    Not used directly; detection runs at scan callback rate (~10Hz).
"""

import math
import json
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class WallDetectorNode(Node):
    """
    Processes LiDAR scans to extract right-side wall distance.
    Implements sector slicing, validity filtering, and median filtering.
    """

    def __init__(self):
        super().__init__('wall_detector')

        # ── PARAMETER DECLARATIONS ─────────────────────────────────────
        # Declare all parameters with defaults; can be overridden in YAML.

        # Centre of detection sector (degrees, robot frame: right = -90°)
        self.declare_parameter('sector_centre_deg', -90.0)

        # Half-width of detection sector (degrees, total = 2 × half_width)
        self.declare_parameter('sector_half_deg', 30.0)

        # Distance beyond which a reading is treated as a wall gap
        self.declare_parameter('outlier_threshold', 3.0)

        # Minimum number of valid samples required for wall detection
        self.declare_parameter('min_valid_readings', 5)

        # Maximum allowed wall distance (used by state machine threshold)
        self.declare_parameter('max_wall_distance', 2.5)

        # ── FETCH PARAMETERS ───────────────────────────────────────────
        sector_centre_deg = self.get_parameter('sector_centre_deg').value
        sector_half_deg   = self.get_parameter('sector_half_deg').value
        self.outlier_threshold  = self.get_parameter('outlier_threshold').value
        self.min_valid_readings = self.get_parameter('min_valid_readings').value
        self.max_wall_distance  = self.get_parameter('max_wall_distance').value

        # Convert sector bounds to radians for fast comparison
        # sector_min_rad is the smaller (more negative) angle
        # sector_max_rad is the larger angle
        centre_rad             = math.radians(sector_centre_deg)
        half_rad               = math.radians(sector_half_deg)
        self.sector_min_rad    = centre_rad - half_rad   # ≈ -2π/3 (-120°)
        self.sector_max_rad    = centre_rad + half_rad   # ≈ -π/3  (-60°)

        self.get_logger().info(
            f'Wall detection sector: [{math.degrees(self.sector_min_rad):.1f}°, '
            f'{math.degrees(self.sector_max_rad):.1f}°]'
        )

        # ── QoS PROFILE ────────────────────────────────────────────────
        # LaserScan is published with BEST_EFFORT reliability by ros_gz_bridge.
        # We must match this, otherwise the subscription receives nothing.
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── SUBSCRIBERS ────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        # ── PUBLISHERS ─────────────────────────────────────────────────
        # Wall distance: consumed by PID controller
        self.wall_dist_pub = self.create_publisher(
            Float64,
            '/wall_distance',
            10
        )

        # Wall info: consumed by state machine and diagnostics
        self.wall_info_pub = self.create_publisher(
            String,
            '/wall_info',
            10
        )

        # ── INTERNAL STATE ─────────────────────────────────────────────
        # Last valid wall distance (kept for continuity between scans)
        self._last_valid_distance = -1.0
        self._scan_count = 0

        self.get_logger().info('WallDetectorNode initialised.')

    # ──────────────────────────────────────────────────────────────────
    # SCAN CALLBACK
    # Called every time a new LaserScan message arrives (~10 Hz from GZ).
    # ──────────────────────────────────────────────────────────────────
    def scan_callback(self, scan_msg: LaserScan) -> None:
        """
        Main processing callback for LiDAR scan data.

        Algorithm:
          1. Iterate over all rays in the scan.
          2. Compute each ray's angle using:
               angle = angle_min + i * angle_increment
          3. Check if angle falls within the detection sector.
          4. Validate the range reading.
          5. Collect valid readings → apply median filter.
          6. Publish result.

        Args:
            scan_msg: sensor_msgs/LaserScan from /scan topic.
        """
        self._scan_count += 1

        # ── STEP 1: Extract sector readings ───────────────────────────
        # angle_min: the angle of the first ray (radians)
        # angle_increment: angular step between consecutive rays (radians)
        # Both are contained in the LaserScan header.
        #
        # For our 360° LiDAR with 360 samples:
        #   angle_min       = -π   (-180°)
        #   angle_max       = +π   (+180°)
        #   angle_increment =  2π/360 = 0.01745 rad (1°)

        sector_readings = []  # valid range values in the detection sector

        n_rays = len(scan_msg.ranges)  # total number of rays in scan

        for i, range_val in enumerate(scan_msg.ranges):
            # Compute this ray's angle in robot frame
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # ── STEP 2: Sector filter ──────────────────────────────────
            # Only process rays pointing to the right-side sector
            if not (self.sector_min_rad <= angle <= self.sector_max_rad):
                continue

            # ── STEP 3: Range validity filter ─────────────────────────
            # Discard NaN (no return) and Inf (out of range)
            if not math.isfinite(range_val):
                continue

            # Discard readings outside the sensor's valid range
            # LaserScan provides range_min and range_max from the sensor spec
            if range_val < scan_msg.range_min:
                continue  # too close (crosstalk / noise)
            if range_val > scan_msg.range_max:
                continue  # beyond sensor range

            # ── STEP 4: Outlier filter ─────────────────────────────────
            # Readings above outlier_threshold (3.0m default) indicate
            # a gap in the wall (e.g. rack aisle, corridor junction).
            # We do NOT discard these completely — we track them for
            # gap detection — but we exclude them from the median calc.
            if range_val > self.outlier_threshold:
                continue

            sector_readings.append(range_val)

        # ── STEP 5: Compute wall distance using median filter ──────────
        n_valid = len(sector_readings)
        wall_detected = n_valid >= self.min_valid_readings

        if wall_detected:
            # Median is robust to spikes (specular reflections, edges).
            # statistics.median() returns a float for even-length lists
            # by averaging the two central values.
            wall_distance = statistics.median(sector_readings)

            # Sanity-check: distance must be positive
            if wall_distance <= 0.0:
                wall_detected = False
                wall_distance = -1.0
            else:
                self._last_valid_distance = wall_distance
        else:
            # Wall not detected in this scan; return sentinel value
            wall_distance = -1.0

        # ── STEP 6: Publish wall distance ──────────────────────────────
        dist_msg = Float64()
        dist_msg.data = wall_distance
        self.wall_dist_pub.publish(dist_msg)

        # ── STEP 7: Publish diagnostic info ────────────────────────────
        if wall_detected and len(sector_readings) > 0:
            info_dict = {
                'distance':      round(wall_distance, 4),
                'detected':      wall_detected,
                'valid_samples': n_valid,
                'sector_min':    round(min(sector_readings), 4),
                'sector_max':    round(max(sector_readings), 4),
                'total_rays':    n_rays
            }
        else:
            info_dict = {
                'distance':      -1.0,
                'detected':      False,
                'valid_samples': n_valid,
                'sector_min':    -1.0,
                'sector_max':    -1.0,
                'total_rays':    n_rays
            }

        info_msg = String()
        info_msg.data = json.dumps(info_dict)
        self.wall_info_pub.publish(info_msg)

        # ── DEBUG LOGGING (every 20 scans = ~2 seconds) ────────────────
        if self._scan_count % 20 == 0:
            if wall_detected:
                self.get_logger().debug(
                    f'Wall detected: {wall_distance:.3f}m '
                    f'({n_valid} samples), '
                    f'range [{min(sector_readings):.3f}, '
                    f'{max(sector_readings):.3f}]'
                )
            else:
                self.get_logger().debug(
                    f'Wall NOT detected (only {n_valid} valid samples). '
                    f'Last known: {self._last_valid_distance:.3f}m'
                )


def main(args=None):
    """Entry point for ros2 run warehouse_wall_follower wall_detector."""
    rclpy.init(args=args)

    node = WallDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WallDetectorNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
