#!/usr/bin/env python3

import math
import threading
from typing import Dict, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
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
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class OccupancyGridBuilderTF(Node):
    """Build a 2D occupancy grid from a point cloud using TF."""

    def __init__(self):
        super().__init__("occupancy_grid_builder_tf")

        # Sim time (robust)
        try:
            self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # Frames / topics
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.cloud_topic = self.declare_parameter("cloud_topic", "/R1/camera/points").value
        self.pose_topic = self.declare_parameter("pose_topic", "/R1/robot_pose_slam").value

        self.map_topic = self.declare_parameter("map_topic", "/occupancy_grid").value
        self.map_updates_topic = self.declare_parameter("map_updates_topic", "/occupancy_grid_updates").value

        # Grid params
        self.resolution = float(self.declare_parameter("resolution", 0.5).value)  # meters/cell
        self.grid_size = int(self.declare_parameter("grid_size", 1000).value)      # cells (square)

        # Map origin: world coords of cell (0,0)
        # If you don't know, set origin_offset big enough so all x/y fall inside.
        self.origin_offset = float(self.declare_parameter("origin_offset", 300.0).value)

        # Point filtering (after TF transform into map frame)
        self.z_min = float(self.declare_parameter("z_min", -2.0).value)
        self.z_max = float(self.declare_parameter("z_max", 5.0).value)
        self.r_min = float(self.declare_parameter("range_min", 0.1).value)
        self.r_max = float(self.declare_parameter("range_max", 200.0).value)

        # Inflation around hits (cells)
        self.inflation_radius = int(self.declare_parameter("inflation_radius", 1).value)

        # Downsample
        self.stride = int(self.declare_parameter("stride", 10).value)
        self.max_points = int(self.declare_parameter("max_points", 50000).value)

        # Accumulation / publish
        self.hit_threshold = int(self.declare_parameter("hit_threshold", 1).value)
        self.publish_every = int(self.declare_parameter("publish_every", 2).value)
        self._cloud_count = 0

        # Optional free-space raytracing (OFF by default because it fills the map quickly)
        self.enable_raytrace = bool(self.declare_parameter("enable_raytrace", False).value)
        self.raytrace_step_m = float(self.declare_parameter("raytrace_step_m", 0.5).value)

        # Storage
        self._lock = threading.Lock()
        self.hits = np.zeros((self.grid_size, self.grid_size), dtype=np.uint16)
        self.occ = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)  # -1 unknown

        # Latest pose (optional)
        self.latest_pose_xy: Optional[Tuple[float, float]] = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS: point clouds are usually BEST_EFFORT in bags; allow both.
        sub_qos = QoSProfile(
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

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, sub_qos)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, pose_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, pub_qos)
        self.map_updates_pub = self.create_publisher(OccupancyGrid, self.map_updates_topic, pub_qos)

        self.get_logger().info(
            f"GridBuilderTF: cloud={self.cloud_topic} pose={self.pose_topic} -> {self.map_topic} "
            f"{self.grid_size}x{self.grid_size} @ {self.resolution}m, origin_offset={self.origin_offset}"
        )

    # -------------------- grid helpers --------------------

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x + self.origin_offset) / self.resolution)
        gy = int((y + self.origin_offset) / self.resolution)
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def mark_occupied(self, gx: int, gy: int):
        if not self.in_bounds(gx, gy):
            return
        self.occ[gy, gx] = 100
        if self.inflation_radius <= 0:
            return
        r = self.inflation_radius
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx, ny = gx + dx, gy + dy
                if self.in_bounds(nx, ny):
                    self.occ[ny, nx] = 100

    def mark_free_line(self, x0: float, y0: float, x1: float, y1: float):
        """Very cheap free-space marking along a line in map frame."""
        step = max(self.raytrace_step_m, 1e-3)
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return
        n = int(dist / step)
        for i in range(n):
            t = i / float(max(n, 1))
            x = x0 + t * dx
            y = y0 + t * dy
            gx, gy = self.world_to_grid(x, y)
            if self.in_bounds(gx, gy) and self.occ[gy, gx] != 100:
                self.occ[gy, gx] = 0

    # -------------------- callbacks --------------------

    def pose_cb(self, msg: PoseStamped):
        with self._lock:
            self.latest_pose_xy = (msg.pose.position.x, msg.pose.position.y)

    def cloud_cb(self, msg: PointCloud2):
        # Lookup TF: map <- cloud_frame at stamp (fallback: latest)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed {self.map_frame}<-{msg.header.frame_id}: {e}")
                return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)

        # Read points (downsample + cap)
        pts = []
        for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if self.stride > 1 and (i % self.stride) != 0:
                continue
            # p can be tuple or numpy.void (structured). Force floats.
            pts.append((float(p[0]), float(p[1]), float(p[2])))
            if len(pts) >= self.max_points:
                break
        if not pts:
            return

        pts_np = np.asarray(pts, dtype=np.float64)  # Nx3 in cloud frame
        pts_map = (pts_np @ R.T) + trans            # Nx3 in map frame

        # Filter by Z and range (range computed in XY relative to camera position)
        dx = pts_map[:, 0] - trans[0]
        dy = pts_map[:, 1] - trans[1]
        rng = np.sqrt(dx * dx + dy * dy)
        m = (
            (rng >= self.r_min)
            & (rng <= self.r_max)
            & (pts_map[:, 2] >= self.z_min)
            & (pts_map[:, 2] <= self.z_max)
        )
        pts_map = pts_map[m]
        if pts_map.shape[0] == 0:
            return

        # Update grid
        with self._lock:
            cam_x, cam_y = float(trans[0]), float(trans[1])
            for x, y, _z in pts_map:
                gx, gy = self.world_to_grid(float(x), float(y))
                if not self.in_bounds(gx, gy):
                    continue

                v = int(self.hits[gy, gx])
                if v < 65535:
                    self.hits[gy, gx] = v + 1

                if int(self.hits[gy, gx]) >= self.hit_threshold:
                    self.mark_occupied(gx, gy)

                if self.enable_raytrace and self.latest_pose_xy is not None:
                    # Use latest pose if you prefer; or camera position
                    self.mark_free_line(cam_x, cam_y, float(x), float(y))

        # Publish periodically
        self._cloud_count += 1
        if (self._cloud_count % self.publish_every) == 0:
            self.publish(msg.header.stamp)

    # -------------------- publishing --------------------

    def publish(self, stamp):
        with self._lock:
            grid = self.occ.copy()

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = float(-self.origin_offset)
        msg.info.origin.position.y = float(-self.origin_offset)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.reshape(-1, order="C").tolist()

        self.map_pub.publish(msg)
        self.map_updates_pub.publish(msg)


def main():
    rclpy.init()
    node = OccupancyGridBuilderTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
