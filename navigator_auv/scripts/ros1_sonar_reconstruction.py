#!/usr/bin/env python3
"""ROS 2 port of the AIRLabIISc/EROAS (ROS 1) sonar_reconstruction.py.

Turns the forward-looking sonar image into the obstacle points that
ros1_velocity_cbf.py's CBF uses (/rexrov2/point_cloud), exactly as the
original: the closest range row with an echo above the threshold, one point
for the left half of the fan and one for the right, placed using the sonar
pivot angle and transformed to the world frame.

Only the ROS API is changed, plus (marked ADAPTED) what the ROS 2 sonar needs.
"""

import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from marine_acoustic_msgs.msg import ProjectedSonarImage
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

from ros1_sonar_common import sonar_image_uint8


class SonarIntensityPublisher(Node):
    def __init__(self):
        super().__init__('sonar_intensity_publisher')
        # ADAPTED: gain mapping DAVE's FLOAT32 dB image to the UINT8 scale
        # (see ros1_sonar_common.sonar_image_uint8).
        self.sonar_float_gain = float(self.declare_parameter('sonar_float_gain', 0.04).value)

        self.joint_states_sub = Subscriber(self, JointState, '/rexrov2/joint_states')
        self.height_sub = Subscriber(self, Odometry, '/rexrov2/pose_gt')
        self.sonar_sub = Subscriber(
            self, ProjectedSonarImage, '/rexrov2/blueview_p900/sonar_image_raw',
            qos_profile=qos_profile_sensor_data)

        # Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.joint_states_sub, self.height_sub, self.sonar_sub], queue_size=100, slop=1)
        self.sync.registerCallback(self.synchronized_callback)

        # Publisher for the processed point cloud
        self.publisher = self.create_publisher(PointCloud2, '/rexrov2/point_cloud', 10)

        # Initialize variables
        self.pivot_angle = 0.0  # Initialize pivot angle

    def synchronized_callback(self, joint_states_msg, height_msg, sonar_msg):
        try:
            if 'rexrov2/sonar_vertical_joint' in joint_states_msg.name:
                index = joint_states_msg.name.index('rexrov2/sonar_vertical_joint')
                self.pivot_angle = joint_states_msg.position[index]
            self.z_position = height_msg.pose.pose.position.z
            self.y_position = height_msg.pose.pose.position.y
            self.x_position = height_msg.pose.pose.position.x
            self.quat = height_msg.pose.pose.orientation
            self.translation_vector = [self.x_position, self.y_position, self.z_position]

            self.process_sonar_data(sonar_msg)

        except Exception as e:
            self.get_logger().error(f'Error in synchronized callback: {e}')

    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """
        Convert polar coordinates (i, j) to Cartesian coordinates in rover frame.
        The image appears rotated by 90 degrees, so adjust accordingly.
        """  # this is in local frame of the robot
        angle_per_beam = 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi / 4
        distance = j * 15 / max_bins
        x = distance * math.cos(angle_rad)  # distance side ways
        y = distance * math.sin(angle_rad) * math.cos(self.pivot_angle)  # distance to front of rover
        z = distance * math.sin(angle_rad) * math.sin(self.pivot_angle)  # Use the pivot angle as the z-coordinate
        return y, -x, z

    def normalize_quaternion(self, q):
        norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        return q.w / norm, q.x / norm, q.y / norm, q.z / norm

    def quaternion_to_rotation_matrix(self, q):
        w, x, y, z = self.normalize_quaternion(q)
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
        ])
        return R

    def local_to_global(self, local_points, rotation_matrix, translation_vector, additional_translation):
        global_points = []
        for point in local_points:
            local_point = np.array(point)
            global_point = rotation_matrix @ local_point + translation_vector
            global_points.append(global_point)
        return np.array(global_points)

    def process_sonar_data(self, msg):
        # ADAPTED: decode FLOAT32 (DAVE) or UINT8 (NPS/synthetic) to the
        # original UINT8 intensity scale; the original read UINT8 directly.
        data_array = sonar_image_uint8(msg, self.sonar_float_gain)
        no_of_beams = msg.image.beam_count or 512  # ADAPTED: was fixed at 512
        range_bin = len(msg.ranges) or 598  # ADAPTED: was fixed at 598

        # Process the sonar data to calculate the average intensity
        points_contour = self.find_contour_points(data_array, no_of_beams, range_bin)

        # Convert to Cartesian coordinates
        points_cartesian = [
            self.polar_to_cartesian(i, j, no_of_beams, range_bin)
            for (i, j) in points_contour]  # local points wrt rover

        rotation_matrix = self.quaternion_to_rotation_matrix(self.quat)
        # Transform points from local to global coordinates
        additional_translation = [0, 0, 0]
        global_points = self.local_to_global(
            points_cartesian, rotation_matrix, self.translation_vector, additional_translation)

        # Create and publish PointCloud2 message
        header = msg.header
        header.frame_id = 'rexrov2/base_link'
        point_cloud_msg = create_cloud_xyz32(header, global_points.reshape(-1, 3))
        self.publisher.publish(point_cloud_msg)

    def find_contour_points(self, data, no_of_beams=512, range_bin=598):
        range_window = 3
        threshold = 15

        contour_points = []

        data = np.array(data).reshape((range_bin, no_of_beams))

        break_outer_loop = False

        # iterate thru range
        for j in range(1, range_bin - 100, 5):
            if break_outer_loop:
                break
            # iterate thru beam
            for i in range(10, 256, 10):
                window_intensities = data[j:j + range_window, i]
                window_average_intensity = np.mean(window_intensities)
                if window_average_intensity > threshold:
                    contour_points.append((i, j))
                    break_outer_loop = True
                    break
            for i in range(256, 500, 10):
                window_intensities = data[j:j + range_window, i]
                window_average_intensity = np.mean(window_intensities)
                if window_average_intensity > threshold:
                    contour_points.append((i, j))
                    break_outer_loop = True
                    break

        return contour_points


def main():
    rclpy.init()
    node = SonarIntensityPublisher()
    node.get_logger().info('Sonar Intensity Publisher Node Running')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
