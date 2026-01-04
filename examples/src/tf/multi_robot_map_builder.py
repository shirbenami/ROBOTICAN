#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import math
from collections import defaultdict
import threading
import yaml

class EfficientOccupancyGridMapper(Node):
    """
    A real-time 2D occupancy grid generator for multi-robot SLAM systems using 3D point clouds.

    This ROS2 node constructs and maintains a global 2D occupancy grid by projecting filtered 3D
    point clouds from multiple robots. It performs obstacle detection, visibility ray-tracing,
    and robot-aware filtering to improve accuracy and robustness.

    Key Features:
    -------------
    - Subscribes to:
        * /merged_map (PointCloud2): Merged 3D map from all robots.
        * /robot_<id>/robot_pose_slam (PoseStamped): Per-robot SLAM pose for position and heading.
    - Publishes:
        * /occupancy_grid (OccupancyGrid): 2D occupancy map with unknown, free, and occupied states.
        * /robot_grid_positions (PoseArray): Per-robot grid cell positions and headings (encoded in pose).
    - Supports:
        * Robot body filtering to ignore robot parts as obstacles
        * Free-space marking along robot paths and within their field-of-view (FOV)
        * Efficient ray-casting using Bresenham’s algorithm
        * Configurable per-robot transforms for frame alignment

    Parameters:
    -----------
    - robot_configs (str, YAML): Dictionary mapping robot IDs to initial positions and orientations.

    Internal Map Representation:
    ----------------------------
    - Grid resolution: 0.5 meters per cell (configurable)
    - Map size: 30m x 30m centered around origin
    - Occupancy values: -1 = unknown, 0 = free, 1 = occupied
    - Point density threshold to mark occupied space
    - Robot-safe zones and open area heuristics to reduce false positives

    Use Case:
    ---------
    This node should be launched in systems using ORB-SLAM3, map merging, and multi-agent exploration.
    It enables all agents to operate with a consistent shared 2D map while avoiding marking robots
    themselves as obstacles.
    """


    def __init__(self):
        super().__init__('efficient_occupancy_grid_mapper')

        # Declare and read robot_configs parameter
        self.declare_parameter('robot_configs', '{}')
        robot_configs_yaml = self.get_parameter('robot_configs').value
        self.robot_configs = yaml.safe_load(robot_configs_yaml)

        # Extract robot IDs from config
        self.robot_ids = list(self.robot_configs.keys())

        # ======================
        # Map Parameters
        # ======================
        self.resolution = 0.5  # 10cm cells for good balance
        self.map_size_meters = 30.0  # Total map size (30m x 30m)
        self.grid_size = int(self.map_size_meters / self.resolution)
        self.origin_offset = self.map_size_meters / 2.0

        # Height filter for 2D projection
        self.z_min = 0.1  # Minimum height to consider
        self.z_max = 3.5  # Maximum height to consider

        # Occupancy thresholds
        self.points_threshold = 15  # Min points in cell to mark as occupied
        self.max_range = 8.0  # Maximum sensor range in meters
        self.open_area_range = 2.0

        # Camera FOV parameters
        self.camera_fov = math.radians(60)  # 60 degree FOV
        self.fov_resolution = math.radians(1)  # Angular resolution for FOV scanning

        # Robot filtering parameters
        self.robot_radius = 0.5 # Radius around robot to ignore points (meters)
        self.robot_clearance_radius = 0.5  # Radius for guaranteed free space around robots

        # ======================
        # Data Structures
        # ======================
        # Main occupancy grid: 0=free, 1=occupied, -1=unknown
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Point count grid for efficient density calculation
        self.point_count = np.zeros((self.grid_size, self.grid_size), dtype=np.uint16)

        # Track cells that robots have traversed (guaranteed free)
        self.traversed_cells = set()

        # Robot states
        self.robot_poses = {rid: None for rid in self.robot_ids}
        self.robot_grid_poses = {rid: None for rid in self.robot_ids}
        self.robot_headings = {rid: 0.0 for rid in self.robot_ids}

        # Transformation matrices
        self.robot_transforms = {}
        for robot_id, config in self.robot_configs.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config.get('orientation', [0, 0, 0])
            )

        # Mark initial positions as free
        self.mark_initial_positions_free()

        # Thread safety
        self.lock = threading.Lock()

        # ======================
        # Publishers & Subscribers
        # ======================
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/occupancy_grid', 10
        )

        # Publisher for all robot positions in grid coordinates
        self.robot_positions_pub = self.create_publisher(
            PoseArray, '/robot_grid_positions', 10
        )

        # Subscribe to merged point cloud
        self.create_subscription(
            PointCloud2,
            '/R1/camera/points',
            self.point_cloud_callback,
            10
        )

        # Subscribe to robot poses
        for robot_id in self.robot_ids:
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/robot_pose_slam',
                self.create_pose_callback(robot_id),
                10
            )

        # Timer for map publishing
        self.create_timer(0.5, self.publish_map)  # 2Hz publishing

        # Timer for robot positions publishing
        self.create_timer(0.5, self.publish_robot_positions)  # 10Hz publishing

        self.get_logger().info("Efficient Occupancy Grid Mapper initialized with robot filtering and path tracking")

    def mark_initial_positions_free(self):
        """
        Mark the initial configuration positions of all robots as free space in the occupancy grid.

        This prevents false-positive obstacle markings at the robots' starting locations.
        """
        for robot_id, config in self.robot_configs.items():
            x, y = config['position'][0], config['position'][1]
            gx, gy = self.world_to_grid(x, y)

            # Mark a radius around initial position as free
            radius_cells = int(self.robot_clearance_radius / self.resolution)
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    cell_x, cell_y = gx + dx, gy + dy
                    if self.is_valid_grid_pos(cell_x, cell_y):
                        dist_sq = dx * dx + dy * dy
                        if dist_sq <= radius_cells * radius_cells:
                            self.traversed_cells.add((cell_x, cell_y))
                            self.occupancy_grid[cell_y, cell_x] = 0

    def create_transformation_matrix(self, position, orientation):
        """
        Create a 4x4 transformation matrix from position and orientation (Euler angles).

        Args:
            position (list): [x, y, z] position of robot.
            orientation (list): [roll, pitch, yaw] in radians.

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def world_to_grid(self, x, y):
        """
        Convert world coordinates (in meters) to grid indices.

        Args:
            x (float): World x-coordinate.
            y (float): World y-coordinate.

        Returns:
            tuple: (grid_x, grid_y) indices in the occupancy grid.
        """
        gx = int((x + self.origin_offset) / self.resolution)
        gy = int((y + self.origin_offset) / self.resolution)
        return gx, gy

    def is_valid_grid_pos(self, gx, gy):
        """
        Check if grid cell indices are within the map bounds.

        Args:
            gx (int): Grid x index.
            gy (int): Grid y index.

        Returns:
            bool: True if indices are valid.
        """
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def is_near_any_robot(self, world_x, world_y):
        """
        Determine whether a given point is within a robot's physical body radius.

        Args:
            world_x (float): World x-coordinate of the point.
            world_y (float): World y-coordinate of the point.

        Returns:
            bool: True if point is within robot radius.
        """
        for robot_id, robot_pose in self.robot_poses.items():
            if robot_pose is not None:
                rx, ry = robot_pose
                dist_sq = (world_x - rx) ** 2 + (world_y - ry) ** 2
                if dist_sq <= self.robot_radius ** 2:
                    return True
        return False

    def mark_robot_path_free(self, robot_id, gx, gy):
        """
        Mark a radius around the robot's current grid position as free space.

        The radius is 0.5 meters, which translates to cells based on the current resolution.

        Args:
            robot_id (str): ID of the robot.
            gx (int): Grid x index of robot position.
            gy (int): Grid y index of robot position.
        """
        # Mark a small radius around the robot as definitely free
        radius_cells = int(0.5 / self.resolution)  # 0.5m radius
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                cell_x, cell_y = gx + dx, gy + dy
                if self.is_valid_grid_pos(cell_x, cell_y):
                    self.traversed_cells.add((cell_x, cell_y))
                    self.occupancy_grid[cell_y, cell_x] = 0

    def create_pose_callback(self, robot_id):
        """
        Generate a pose subscriber callback for a given robot.

        The callback updates:
        - Robot world and grid position.
        - Robot heading.
        - Traversed path markings.

        Args:
            robot_id (str): Robot identifier.

        Returns:
            function: Callback to handle incoming PoseStamped messages.
        """
        def callback(msg):
            # Transform to global frame
            local_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                1.0
            ])
            global_pose = self.robot_transforms[robot_id] @ local_pose

            # Store world position
            world_x, world_y = global_pose[0], global_pose[1]

            with self.lock:
                self.robot_poses[robot_id] = (world_x, world_y)

                # Convert to grid
                gx, gy = self.world_to_grid(world_x, world_y)
                if self.is_valid_grid_pos(gx, gy):
                    self.robot_grid_poses[robot_id] = (gx, gy)
                    # Mark robot's path as free
                    self.mark_robot_path_free(robot_id, gx, gy)

                # Calculate heading
                q = msg.pose.orientation
                local_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))

                # Apply transformation rotation
                rot_matrix = self.robot_transforms[robot_id][:3, :3]
                transform_yaw = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])
                self.robot_headings[robot_id] = local_yaw + transform_yaw

        return callback

    def publish_robot_positions(self):
        """
        Publish all robot positions as a `PoseArray`, in grid coordinates.

        The z field of each pose encodes the robot ID (as float).
        """
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        with self.lock:
            for robot_id in sorted(self.robot_ids):  # Sort to ensure consistent order
                if self.robot_grid_poses[robot_id] is not None:
                    pose = Pose()
                    gx, gy = self.robot_grid_poses[robot_id]

                    # Grid coordinates
                    pose.position.x = float(gx)
                    pose.position.y = float(gy)
                    pose.position.z = float(robot_id)  # Encode robot_id in z

                    # Orientation (heading)
                    yaw = self.robot_headings[robot_id]
                    pose.orientation.z = math.sin(yaw / 2.0)
                    pose.orientation.w = math.cos(yaw / 2.0)

                    msg.poses.append(pose)

        self.robot_positions_pub.publish(msg)

    def point_cloud_callback(self, msg):
        """
        Main processing function for the merged point cloud.

        - Filters 3D points based on height and robot proximity.
        - Populates a point count grid.
        - Marks high-density cells as occupied.
        - Performs visibility-based ray tracing for free space.
        """
        try:
            # Extract points
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if self.z_min <= p[2] <= self.z_max:  # Height filter
                    # Filter out points near any robot
                    if not self.is_near_any_robot(p[0], p[1]):
                        points.append((p[0], p[1]))

            with self.lock:
                # Reset point count
                self.point_count.fill(0)

                # Count points per cell (very fast operation)
                for x, y in points:
                    gx, gy = self.world_to_grid(x, y)
                    if self.is_valid_grid_pos(gx, gy):
                        # Don't count points in traversed cells
                        if (gx, gy) not in self.traversed_cells:
                            self.point_count[gy, gx] += 1

                # Mark cells as occupied based on point density
                occupied_cells = set()
                for gy in range(self.grid_size):
                    for gx in range(self.grid_size):
                        # Never mark traversed cells as occupied
                        if (gx, gy) in self.traversed_cells:
                            continue

                        if self.point_count[gy, gx] >= self.points_threshold:
                            self.occupancy_grid[gy, gx] = 1
                            occupied_cells.add((gx, gy))

                # Update free space using efficient ray casting
                self.update_free_space_fast(occupied_cells)

                # NEW: Mark open areas in the field of view even when no obstacles are detected
                self.mark_open_areas_in_fov()

        except Exception as e:
            self.get_logger().error(f"Error in point cloud callback: {e}")

    def mark_open_areas_in_fov(self):
        """
        Mark unknown cells as free within the robot's field of view (FOV),
        when no obstacles are detected.

        This helps fill in gaps in open spaces.
        """
        for robot_id, robot_grid_pos in self.robot_grid_poses.items():
            if robot_grid_pos is None:
                continue

            rx, ry = robot_grid_pos
            robot_heading = self.robot_headings[robot_id]

            # Scan through the FOV
            left_angle = robot_heading - self.camera_fov / 2
            right_angle = robot_heading + self.camera_fov / 2

            # Angular steps through FOV
            angle = left_angle
            while angle <= right_angle:
                # Check along this ray direction
                obstacle_found = False
                obstacle_distance = self.max_range

                # First, check if there's an obstacle along this ray
                max_cells = int(self.max_range / self.resolution)
                for dist_cells in range(1, max_cells + 1):
                    dist_meters = dist_cells * self.resolution

                    # Calculate grid position
                    check_x = int(rx + dist_cells * math.cos(angle))
                    check_y = int(ry + dist_cells * math.sin(angle))

                    if self.is_valid_grid_pos(check_x, check_y):
                        if self.occupancy_grid[check_y, check_x] == 1:  # Obstacle found
                            obstacle_found = True
                            obstacle_distance = dist_meters
                            break

                # Now mark cells as free based on what we found
                if obstacle_found:
                    # If obstacle found, we already marked free cells up to it in update_free_space_fast
                    pass
                else:
                    # No obstacle found - mark cells as free up to half the sensor range
                    free_range_cells = int(self.open_area_range / self.resolution)
                    for dist_cells in range(1, free_range_cells + 1):
                        free_x = int(rx + dist_cells * math.cos(angle))
                        free_y = int(ry + dist_cells * math.sin(angle))

                        if self.is_valid_grid_pos(free_x, free_y):
                            if self.occupancy_grid[free_y, free_x] == -1:  # Only mark unknown cells
                                self.occupancy_grid[free_y, free_x] = 0  # Mark as free

                angle += self.fov_resolution

    def update_free_space_fast(self, occupied_cells):
        """
        Perform fast ray tracing from each robot to observed obstacles using Bresenham’s algorithm.

        This clears free space between the robot and each obstacle (line-of-sight visibility).

        Args:
            occupied_cells (set): Set of (gx, gy) cells marked as occupied.
        """
        for robot_id, robot_grid_pos in self.robot_grid_poses.items():
            if robot_grid_pos is None:
                continue

            rx, ry = robot_grid_pos

            # Process only occupied cells within sensor range
            max_cells = int(self.max_range / self.resolution)

            for (ox, oy) in occupied_cells:
                # Check if occupied cell is within range
                dx = ox - rx
                dy = oy - ry
                dist_sq = dx * dx + dy * dy

                if dist_sq <= max_cells * max_cells:
                    # Check if within FOV
                    angle_to_obstacle = math.atan2(dy, dx)
                    robot_heading = self.robot_headings[robot_id]
                    angle_diff = abs(self.normalize_angle(angle_to_obstacle - robot_heading))

                    if angle_diff <= self.camera_fov / 2:
                        # Trace ray from robot to occupied cell
                        self.trace_ray_bresenham(rx, ry, ox, oy)

    def normalize_angle(self, angle):
        """
        Normalize an angle to the [-pi, pi] range.

        Args:
            angle (float): Input angle in radians.

        Returns:
            float: Normalized angle.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def trace_ray_bresenham(self, x0, y0, x1, y1):
        """
        Apply Bresenham's line algorithm to trace a ray between two grid cells.

        Marks all intermediate cells as free unless already occupied or traversed.

        Args:
            x0 (int): Start grid x index (robot position).
            y0 (int): Start grid y index.
            x1 (int): End grid x index (obstacle).
            y1 (int): End grid y index.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            # Don't mark the robot's cell or the final cell as free
            if (x, y) != (x0, y0) and (x, y) != (x1, y1):
                if self.is_valid_grid_pos(x, y) and self.occupancy_grid[y, x] != 1:
                    # Don't override traversed cells
                    if (x, y) not in self.traversed_cells:
                        self.occupancy_grid[y, x] = 0  # Mark as free

            # Check if we've reached the end
            if x == x1 and y == y1:
                break

            # Bresenham's algorithm step
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_map(self):
        """
        Publish the current occupancy grid as a ROS `OccupancyGrid` message.

        Unknown = -1, Free = 0, Occupied = 100.
        """
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.origin_offset
        msg.info.origin.position.y = -self.origin_offset
        msg.info.origin.position.z = 0.0

        # Convert to ROS format: unknown=-1, free=0, occupied=100
        with self.lock:
            data = []
            for y in range(self.grid_size):
                for x in range(self.grid_size):
                    val = self.occupancy_grid[y, x]
                    if val == -1:
                        data.append(-1)
                    elif val == 0:
                        data.append(0)
                    else:  # val == 1
                        data.append(100)

            msg.data = data

        self.map_pub.publish(msg)

        # Log statistics occasionally
        if hasattr(self, '_last_log_time'):
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_log_time > 5.0:  # Log every 5 seconds
                # self.log_statistics()
                self._last_log_time = current_time
        else:
            self._last_log_time = self.get_clock().now().nanoseconds / 1e9

    def log_statistics(self):
        """
        Log the number of free, occupied, and unknown cells in the map.

        This is called periodically and intended for monitoring/debugging.
        """
        with self.lock:
            unknown = np.sum(self.occupancy_grid == -1)
            free = np.sum(self.occupancy_grid == 0)
            occupied = np.sum(self.occupancy_grid == 1)
            total = self.grid_size * self.grid_size

            self.get_logger().info(
                f"Map stats - Unknown: {unknown} ({100 * unknown / total:.1f}%), "
                f"Free: {free} ({100 * free / total:.1f}%), "
                f"Occupied: {occupied} ({100 * occupied / total:.1f}%)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = EfficientOccupancyGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()