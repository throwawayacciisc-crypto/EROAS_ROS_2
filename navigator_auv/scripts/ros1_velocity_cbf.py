#!/usr/bin/env python3
"""ROS 2 port of the AIRLabIISc/EROAS (ROS 1) velocity_cbf.py.

ST-CBF safety filter between the SPD2C planner (/rexrov2/cmd_vel_1) and the
vehicle (/rexrov2/cmd_vel). Obstacles are the sonar-image points published by
ros1_sonar_reconstruction.py on /rexrov2/point_cloud, accumulated within
`radius` of the vehicle; h = d_min^2 - R_o^2 for the closest one.

Only the ROS API is changed, plus (marked ADAPTED):
- cvxpy is replaced by the closed-form solution of the same QP. The problem
  is min ||v - v_des||^2 s.t. a.v + b >= 0 (two variables, one linear
  constraint), whose optimum is v_des if it already satisfies the
  constraint, otherwise its projection onto the line a.v + b = 0. Like the
  original's "optimization failed" branch, an infeasible problem (a = 0,
  b < 0) yields zero velocity.
"""

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64


def solve_halfspace_qp(desired, a, b):
    """argmin_v ||v - desired||^2  s.t.  a.v + b >= 0  (None if infeasible)."""
    desired = np.asarray(desired, dtype=float)
    a = np.asarray(a, dtype=float)
    slack = float(a @ desired + b)
    if slack >= 0.0:
        return desired
    norm_sq = float(a @ a)
    if norm_sq <= 1e-12:
        return None
    return desired - (slack / norm_sq) * a


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscribers
        self.vel_sub = self.create_subscription(Twist, '/rexrov2/cmd_vel_1', self.vel_callback, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/rexrov2/point_cloud', self.point_cloud_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10)
        self.sonar_sub = self.create_subscription(
            Float64, '/rexrov2/sonar/moving', self.sonar_callback, 10)
        self.h_pub = self.create_publisher(Float64, '/rexrov2/current_h', 10)
        self.closest_publisher = self.create_publisher(PointCloud2, '/rexrov2/closest_point', 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel', 10)

        # CBF parameters
        self.safe_distance = 0  # Minimum safe distance from obstacles
        self.R_o = 2
        self.radius = 15
        self.kappa = 0.09
        self.kappa1 = 0.09

        self.filtered_points = np.empty((0, 3))
        # Variables
        self.current_pose = None
        self.current_vel = None
        self.point_cloud = None
        self.vehicle_pose = None
        self.current_h = float('inf')
        self.sonar_moving = False
        self.xy_cbf = False
        self.xz_cbf = False
        self.closest_points = []
        self.quaternion = None
        self.closest_point = None
        self.closest_obstacle_distance = float('inf')

    def sonar_callback(self, msg):
        sonar_state = msg.data
        if sonar_state == 1:
            self.sonar_moving = False
            self.xy_cbf = False
            self.xz_cbf = True
            self.R_o = 2
        if sonar_state == 2:
            self.sonar_moving = True
            self.xy_cbf = False
            self.xz_cbf = False
            self.R_o = 4
        if sonar_state == 0:
            self.sonar_moving = False
            self.xy_cbf = True
            self.xz_cbf = False
            self.R_o = 4

    def vel_callback(self, msg):
        self.v_alg = msg
        self.process_data(self.v_alg)

    def point_cloud_callback(self, msg):
        self.last_h = self.current_h

        if self.vehicle_pose is None:
            self.get_logger().info('Vehicle pose not yet received')
            return  # ADAPTED: the original went on to dereference the pose and raised

        pc_data = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        new_points_float = np.asarray(pc_data, dtype=float).reshape(-1, 3)
        new_points = np.round(new_points_float).astype(int)

        # Extract vehicle position
        vx, vy, vz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z

        # Concatenate existing filtered points with new points
        if len(self.filtered_points) > 0 and len(new_points) > 0:
            filtered_points_np = np.array(self.filtered_points)
            all_points = np.vstack((filtered_points_np, new_points))
        elif len(new_points) > 0:
            all_points = np.array(new_points)
        elif len(self.filtered_points) > 0:
            all_points = np.array(self.filtered_points)
        else:
            all_points = np.empty((0, 3))

        # Compute distances for all points
        if len(all_points) > 0:
            all_points = np.unique(all_points, axis=0)
            distances = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
            smallest_distance = np.min(distances) if distances.size > 0 else float('inf')
            self.get_logger().info(f'Smallest distance: {smallest_distance} ')
            if smallest_distance > self.radius:
                smallest_distance = float('inf')

            # Filter points within the radius
            within_radius_mask = distances <= self.radius
            self.filtered_points = all_points[within_radius_mask].tolist()
        else:
            self.filtered_points = np.empty((0, 3))

    def pose_callback(self, msg):
        if self.xy_cbf or self.xz_cbf or self.sonar_moving:
            # Store vehicle pose (assuming position in global frame)
            self.vehicle_pose = msg.pose.pose.position
            orientation_q = msg.pose.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

            self.quaternion = quaternion

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler = tf_transformations.euler_from_quaternion(quaternion)
            self.yaw = euler[2]  # Only yaw is needed for 2D velocity transformations

            vxx, vyy, vzz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
            filtered = np.asarray(self.filtered_points, dtype=float).reshape(-1, 3)
            distance = np.linalg.norm(filtered - np.array([vxx, vyy, vzz]), axis=1)

            if distance.size > 0:
                # Find the smallest distance
                smallest_distance = np.min(distance)
                # Find all indices where the distance equals the smallest distance
                min_indices = np.where(distance == smallest_distance)[0]
                # Choose the first point (or handle multiple points as needed)
                self.closest_point = filtered[min_indices[0]]
            else:
                smallest_distance = float('inf')
                self.closest_point = None

            # If the smallest distance is greater than the radius, set it to infinity
            if smallest_distance > self.radius:
                smallest_distance = float('inf')
                self.closest_point = None

            self.closest_obstacle_distance = smallest_distance

            # Compute current_h, if closest_point is not None
            self.current_h = (self.closest_obstacle_distance**2) - (self.R_o**2)

    def h_state_dot_xy(self):
        return np.array([
            2 * (self.vehicle_pose.x - self.closest_point[0]),
            2 * (self.vehicle_pose.y - self.closest_point[1])])

    def compute_h_dot_xy(self, Vel_1, Vel_2):
        # Time derivative of the CBF
        h_state_dot = self.h_state_dot_xy()
        self.h_dot_xy_t = np.dot(h_state_dot, np.transpose(np.array([Vel_1, Vel_2])))
        return self.h_dot_xy_t

    def quaternion_to_rotation_matrix(self, quaternion):
        x, y, z, w = quaternion
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])

    def global_to_local_3d(self, global_coord):
        vehicle_position = self.vehicle_pose
        orientation_q = self.quaternion
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation_q)
        global_x, global_y, global_z = global_coord
        vehicle_x, vehicle_y, vehicle_z = vehicle_position.x, vehicle_position.y, vehicle_position.z
        shifted_global = np.array([global_x - vehicle_x, global_y - vehicle_y, global_z - vehicle_z])
        local_coord = rotation_matrix.T @ shifted_global
        return local_coord

    def h_state_dot_xz(self):
        local_cordi = self.global_to_local_3d(self.closest_point)
        return np.array([2 * (-local_cordi[0]), 2 * (-local_cordi[2])])

    def compute_h_dot_xz(self, Vel_1, Vel_2):
        h_state_dot = self.h_state_dot_xz()
        self.h_dot_xz_t = np.dot(h_state_dot, np.transpose(np.array([Vel_1, Vel_2])))
        return self.h_dot_xz_t

    def transform_velocity_to_global_xy(self, local_velocity_x, local_velocity_y):
        global_velocity_x = local_velocity_x * np.cos(self.yaw) - local_velocity_y * np.sin(self.yaw)
        global_velocity_y = local_velocity_x * np.sin(self.yaw) + local_velocity_y * np.cos(self.yaw)
        return global_velocity_x, global_velocity_y

    def transform_velocity_to_local_xy(self, global_velocity_x, global_velocity_y):
        local_velocity_x = global_velocity_x * np.cos(self.yaw) + global_velocity_y * np.sin(self.yaw)
        local_velocity_y = -global_velocity_x * np.sin(self.yaw) + global_velocity_y * np.cos(self.yaw)
        return local_velocity_x, local_velocity_y

    def cbf_optimization_xy(self, velll):
        desired_velocity_x, desired_velocity_y = self.transform_velocity_to_global_xy(
            velll.linear.x, velll.linear.y)

        # Constraint: h_dot(v) + kappa * (h - 0.5) >= 0 (ADAPTED: closed-form QP)
        solution = solve_halfspace_qp(
            (desired_velocity_x, desired_velocity_y),
            self.h_state_dot_xy(), self.kappa * (self.current_h - 0.5))

        if solution is not None:
            optimized_velocity_x, optimized_velocity_y = self.transform_velocity_to_local_xy(
                solution[0], solution[1])
            safe_velocity = np.array([optimized_velocity_x, optimized_velocity_y])
        else:
            print('Optimization failed: No optimal solution found')
            safe_velocity = np.array([0.0, 0.0])

        return safe_velocity

    def cbf_optimization_xz(self, velll):
        self.get_logger().info('entered xz optimization')

        desired_velocity_x, desired_velocity_z = velll.linear.x, velll.linear.z

        # Constraint: h_dot(v) + kappa1 * (h - 0.5) >= 0 (ADAPTED: closed-form QP)
        solution = solve_halfspace_qp(
            (desired_velocity_x, desired_velocity_z),
            self.h_state_dot_xz(), self.kappa1 * (self.current_h - 0.5))

        if solution is not None:
            optimized_velocity_x, optimized_velocity_z = solution
            safe_velocity = np.array([optimized_velocity_x, optimized_velocity_z])
            self.get_logger().info(f'closest distance: {self.closest_obstacle_distance}')
            self.get_logger().info(f'z alg :{self.v_alg.linear.z}')
            self.get_logger().info(f'z opti :{optimized_velocity_z}')
            self.get_logger().info(f'x alg :{self.v_alg.linear.x}')
            self.get_logger().info(f'x opti :{optimized_velocity_x}')
            print('Optimization successful')
        else:
            print('Optimization failed: No optimal solution found')
            safe_velocity = np.array([0.0, 0.0])

        return safe_velocity

    def process_data(self, valg):
        # Compute safe control input using CBF
        if self.xy_cbf:
            if not self.current_h == float('inf'):
                v_safe = self.cbf_optimization_xy(valg)
                self.publish_safe_control_input_xy(v_safe)
            else:
                v_safe = np.array([valg.linear.x, valg.linear.y])
                self.publish_safe_control_input_xy(v_safe)
        if self.xz_cbf:
            if not self.current_h == float('inf'):
                v_safe = self.cbf_optimization_xz(valg)
                self.publish_safe_control_input_xz(v_safe)
            else:
                v_safe = np.array([valg.linear.x, valg.linear.z])
                self.publish_safe_control_input_xz(v_safe)
        if self.sonar_moving:
            # ADAPTED: the original published nothing while the sonar sweeps
            # (mode 2). gz-sim's VelocityControl keeps applying the last
            # command, so the vehicle kept turning at the final
            # orient_for_3d yaw rate for the whole 50 s sweep (~35 deg off the
            # goal) and then climbed away on that heading. Hold still instead.
            self.cmd_vel_pub.publish(Twist())

    def publish_safe_control_input_xy(self, velocity):
        twist = Twist()
        twist.angular.z = self.v_alg.angular.z  # Set the angular velocity
        twist.linear.x = float(velocity[0])
        twist.linear.y = float(velocity[1])
        self.cmd_vel_pub.publish(twist)

    def publish_safe_control_input_xz(self, velocity):
        twist = Twist()
        self.h_pub.publish(Float64(data=float(self.current_h)))
        twist.linear.x = float(velocity[0])
        twist.linear.z = float(velocity[1])
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
