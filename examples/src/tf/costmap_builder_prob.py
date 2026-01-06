#!/usr/bin/env python3
"""
Probabilistic rolling costmap (2D OccupancyGrid) from a PointCloud2.

- Subscribes:
  * cloud_topic (sensor_msgs/PointCloud2)  e.g. /R1/camera/points or /simple_drone/front/points
  * pose_topic  (geometry_msgs/PoseStamped) e.g. /simple_drone/robot_pose_slam
    (This is your abstraction layer: publish PoseStamped from either Odometry or UAVState elsewhere.)

- Publishes:
  * map_topic (nav_msgs/OccupancyGrid) with values:
      -1 unknown, [0..100] probability of occupancy

Core idea:
  - Build a *rolling/local* grid centered on the robot pose.
  - For each incoming cloud, project to 2D, count points per cell.
  - Convert count -> occupancy probability with a saturating function.
  - (Optional) very cheap free-space raytracing.
"""

import math
import threading
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
            [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield grid cells along a line from (x0,y0) to (x1,y1) (inclusive)."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


class ProbabilisticCostmapBuilder(Node):
    def __init__(self):
        super().__init__("probabilistic_costmap_builder")

        # Sim time (robust)
        try:
            self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # Topics / frames
        self.cloud_topic = self.declare_parameter("cloud_topic", "/R1/camera/points").value
        self.pose_topic = self.declare_parameter("pose_topic", "/simple_drone/odom").value
        self.map_topic = self.declare_parameter("map_topic", "/occupancy_grid").value

        # Fixed/global frame for the costmap (Gazebo: simple_drone/odom)
        self.map_frame = self.declare_parameter("map_frame", "map").value

        # Rolling window params
        self.resolution = float(self.declare_parameter("resolution", 0.30).value)   # m/cell (20-50cm)
        self.size_meters = float(self.declare_parameter("size_meters", 40.0).value) # local window size
        self.width = int(round(self.size_meters / self.resolution))
        self.height = int(round(self.size_meters / self.resolution))

        # Point filtering (after TF into map_frame)
        self.z_min = float(self.declare_parameter("z_min", -0.3).value)
        self.z_max = float(self.declare_parameter("z_max", 3.5).value)
        self.range_min = float(self.declare_parameter("range_min", 0.3).value)
        self.range_max = float(self.declare_parameter("range_max", 20.0).value)

        # Downsample
        self.stride = int(self.declare_parameter("stride", 3).value)
        self.max_points = int(self.declare_parameter("max_points", 120000).value)

        # Probability mapping: count -> p_occ
        self.points_threshold = int(self.declare_parameter("points_threshold", 15).value)
        self.target_prob_at_threshold = float(self.declare_parameter("target_prob_at_threshold", 0.90).value)
        self._alpha = self._compute_alpha(self.points_threshold, self.target_prob_at_threshold)

        # Optional free-space marking (cheap, can be noisy)
        self.enable_raytrace = bool(self.declare_parameter("enable_raytrace", False).value)
        self.raytrace_stride = int(self.declare_parameter("raytrace_stride", 25).value)
        self.occupied_min_count_for_ray = int(self.declare_parameter("occupied_min_count_for_ray", 3).value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self._lock = threading.Lock()
        self.latest_pose_xy: Optional[Tuple[float, float]] = None

        # QoS
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, cloud_qos)
        self.create_subscription(Odometry, self.pose_topic, self.pose_cb, pose_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, pub_qos)

        self.get_logger().info(
            f"ProbCostmap: cloud={self.cloud_topic} pose={self.pose_topic} -> {self.map_topic} "
            f"{self.width}x{self.height} @ {self.resolution:.2f}m, size={self.size_meters:.1f}m, map_frame={self.map_frame}"
        )

    @staticmethod
    def _compute_alpha(threshold: int, target_prob: float) -> float:
        # p = 1 - exp(-alpha * count), choose alpha so p(threshold)=target_prob
        thr = max(int(threshold), 1)
        tp = float(min(max(target_prob, 1e-3), 0.999))
        return -math.log(1.0 - tp) / float(thr)

    def pose_cb(self, msg: Odometry):
        with self._lock:
            self.latest_pose_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def cloud_cb(self, msg: PointCloud2):
        with self._lock:
            pose_xy = self.latest_pose_xy

        if pose_xy is None:
            self.get_logger().warn("No PoseStamped yet; waiting (pose_topic).")
            return

        robot_x, robot_y = pose_xy

        # Rolling origin (robot centered)
        origin_x = robot_x - 0.5 * self.size_meters
        origin_y = robot_y - 0.5 * self.size_meters

        # TF: map_frame <- cloud_frame (strip leading '/')
        # cloud_frame = (msg.header.frame_id or "").lstrip("/")
        cloud_frame = msg.header.frame_id
        if not cloud_frame:
            self.get_logger().warn("PointCloud2 has empty frame_id; cannot TF.")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, cloud_frame, msg.header.stamp, timeout=Duration(seconds=0.2)
            )
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, cloud_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2)
                )
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed {self.map_frame}<-{cloud_frame}: {e}")
                return

        t = tf.transform.translation
        q = tf.transform.rotation
        Rm = quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)

        # Read points (force float tuples -> avoids numpy structured dtype problems)
        pts = []
        for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if self.stride > 1 and (i % self.stride) != 0:
                continue
            pts.append((float(p[0]), float(p[1]), float(p[2])))
            if len(pts) >= self.max_points:
                break
        if not pts:
            return

        pts_np = np.asarray(pts, dtype=np.float64)   # Nx3 in cloud frame
        pts_map = (pts_np @ Rm.T) + trans            # Nx3 in map frame

        # Filter
        dx = pts_map[:, 0] - trans[0]
        dy = pts_map[:, 1] - trans[1]
        rng = np.sqrt(dx * dx + dy * dy)
        m = (
            (rng >= self.range_min) & (rng <= self.range_max) &
            (pts_map[:, 2] >= self.z_min) & (pts_map[:, 2] <= self.z_max)
        )
        pts_map = pts_map[m]
        if pts_map.shape[0] == 0:
            return

        # Bin to cells
        gx = np.floor((pts_map[:, 0] - origin_x) / self.resolution).astype(np.int32)
        gy = np.floor((pts_map[:, 1] - origin_y) / self.resolution).astype(np.int32)
        inb = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        gx = gx[inb]
        gy = gy[inb]
        if gx.size == 0:
            return

        idx = gy * self.width + gx
        counts_1d = np.bincount(idx, minlength=self.width * self.height).astype(np.int32)
        counts = counts_1d.reshape((self.height, self.width))

        p = 1.0 - np.exp(-self._alpha * counts.astype(np.float64))
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        nonzero = counts > 0
        grid[nonzero] = np.clip(np.rint(p[nonzero] * 100.0), 0, 100).astype(np.int8)
        self.get_logger().warn(f"unique counts in grid={np.unique(grid)}")
        # Free-space raytrace (optional)
        if self.enable_raytrace:
            rx = int(math.floor((robot_x - origin_x) / self.resolution))
            ry = int(math.floor((robot_y - origin_y) / self.resolution))
            if 0 <= rx < self.width and 0 <= ry < self.height:
                occ_mask = counts >= int(self.occupied_min_count_for_ray)
                occ_y, occ_x = np.where(occ_mask)
                for k in range(0, occ_x.shape[0], max(1, self.raytrace_stride)):
                    x1 = int(occ_x[k]); y1 = int(occ_y[k])
                    first = True
                    for x, y in bresenham(rx, ry, x1, y1):
                        if first:
                            first = False
                            continue
                        if x == x1 and y == y1:
                            break
                        if grid[y, x] == -1:
                            grid[y, x] = 0

        out = OccupancyGrid()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.info.resolution = float(self.resolution)
        out.info.width = int(self.width)
        out.info.height = int(self.height)
        out.info.origin.position.x = float(origin_x)
        out.info.origin.position.y = float(origin_y)
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = grid.reshape(-1, order="C").tolist()

        self.map_pub.publish(out)
        self.get_logger().info(f"counts nonzero cells={int(np.count_nonzero(nonzero))}")


def main():
    rclpy.init()
    node = ProbabilisticCostmapBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
