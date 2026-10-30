#!/usr/bin/env python3
"""ROS 2 port of the AIRLabIISc/EROAS (ROS 1) only_gap.py (SPD2C planner).

Reads the forward-looking sonar image, finds a >=15 deg obstacle-free gap
closest to the goal bearing, falls back to boundedness turns / contour-slope
following, and switches to the vertical sonar sweep (3D navigation) for flat
obstacles or when out of the goal's depth band. Publishes the desired
velocity on /rexrov2/cmd_vel_1 for ros1_velocity_cbf.py, the sonar pivot
command, and /rexrov2/sonar/moving (0 = 2D, 1 = z motion, 2 = sweeping).

Only the ROS API is changed, plus (marked ADAPTED) what the ROS 2 sonar needs.
"""

import math

import numpy as np
import rclpy
import tf_transformations as tft
from geometry_msgs.msg import Twist
from marine_acoustic_msgs.msg import ProjectedSonarImage
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from ros1_sonar_common import sonar_image_uint8


class SonarHeadingNode(Node):
    def __init__(self):
        super().__init__('sonar_heading_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/rexrov2/cmd_vel_1', 10)
        self.create_subscription(
            ProjectedSonarImage, '/rexrov2/blueview_p900/sonar_image_raw',
            self.sonar_image_raw_callback, qos_profile_sensor_data)
        # Publisher for sonar joint angles
        self.joint_pub = self.create_publisher(
            Float64, '/rexrov2/sonar_joint_position_controller/command', 10)
        self.sonar_move_pub = self.create_publisher(Float64, '/rexrov2/sonar/moving', 10)

        self.create_subscription(Odometry, '/rexrov2/pose_gt', self.pose_callback, 10)

        # ADAPTED: gain mapping DAVE's FLOAT32 dB image to the UINT8 scale
        # (see ros1_sonar_common.sonar_image_uint8).
        self.sonar_float_gain = float(self.declare_parameter('sonar_float_gain', 0.04).value)

        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.criti_cal = False
        self.dominant_slope = None
        self.right_avg_slope = None
        self.left_avg_slope = None
        self.global_angle = 0
        self.turn_around = False
        self.right_goal = False
        self.left_goal = False
        self.avg_right_slope = None
        self.avg_left_slope = None
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.sonar_centered = True
        self.a_poly = None
        self.waypoints = [
            (29, 97, -50),
            (31, 110, -55),
            (30, 90, -90),
            (30, 120, -40),
            # Add more waypoints as needed
        ]
        self.current_goal_index = 0
        self.current_goal = self.waypoints[self.current_goal_index]

        self.data_available = False
        self.collect_3d_data_flag = False
        self.latest_scan_available = False
        self.start_3d_time = None
        self.go_vertical_angle = []
        self.in_z_bound = True
        self.frequency = 0.01
        self.amplitude = 0.7
        self.offset = 0
        self.min_ranges = []
        self.xy_called = False  # to check is process_data called navigate 3d or not, true is process data caled it
        self.min_range = 0
        self.z_motion_ongoing = False
        self.pose_before_z_motion = None
        self.closest_to_target_vertical = None
        self.rospy_first_time = True  # to reset sonar to position 0
        self.ttd = True  # when true moves the sonar from top to bottom
        self.fms = True  # is the first moving sonar scan, before this it should be xy motion
        self.new_midb_available = False
        self.check_verti_go = True  # check for vertical motion for first time, move in vertical only if this flag is false
        self.target_beam = None
        self.orient_3d = False  # will be true until before 3d, then false, will be true is the rover is looking towards goal for 3d
        self.pose = None
        self.beam_count = 512

        # rospy.Rate(8) loop in the original run()
        self.create_timer(1.0 / 8.0, self.run_once)

    def get_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_float(self, publisher, value):
        publisher.publish(Float64(data=float(value)))

    def pose_callback(self, pose_msg):
        self.target_x, self.target_y, self.target_z = self.current_goal
        # Extract robot's pose
        pose = pose_msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation = pose.orientation
        self.pose = pose

        # Convert quaternion to Euler angles
        roll, pitch, yaw = tft.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        # Compute the angle to the target point
        angle_to_target = math.atan2(self.target_y - y, self.target_x - x)

        # Compute the angular error
        angular_error = angle_to_target - yaw

        # Normalize the angular error to be within the range [-pi, pi]
        self.global_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        if (-math.pi / 2) < self.global_angle < (math.pi / 2):
            self.global_angle = math.pi / 2 + self.global_angle
            self.turn_around = False
        elif (math.pi / 2) < self.global_angle < (math.pi):
            self.global_angle = 3.14
            self.turn_around = False
        elif -(math.pi) < self.global_angle < -(math.pi / 2):
            self.global_angle = 0
            self.turn_around = False

        if abs(z - self.target_z) > 5:  # check if z bounded
            self.in_z_bound = False
        else:
            self.in_z_bound = True

        distance_to_goal_xy = math.sqrt((self.target_x - x) ** 2 + (self.target_y - y) ** 2)
        distance_to_goal = math.sqrt(distance_to_goal_xy ** 2 + (self.target_z - z) ** 2)

        if self.collect_3d_data_flag:  # FIND ANGLE TO REACH Z GOAL
            vertical_distance = self.target_z - z
            if distance_to_goal_xy != 0:  # To avoid division by zero
                angle_to_z_goal = math.asin(vertical_distance / distance_to_goal)
            else:
                angle_to_z_goal = math.pi / 2 if vertical_distance > 0 else -math.pi / 2

            self.angle_z_goal = math.degrees(angle_to_z_goal)

        if distance_to_goal < 5.0:
            self.update_goal()

    def update_goal(self):
        # Move to the next waypoint if available
        if self.current_goal_index < len(self.waypoints) - 1:
            self.current_goal_index += 1
            self.current_goal = self.waypoints[self.current_goal_index]
            print(self.current_goal)
            print('Current waypoint reached, going to next')
            print('Current waypoint reached, going to next')
            print('Current waypoint reached, going to next')
        else:
            print('All waypoints have been reached.')
            print('mission over')

    def sonar_image_raw_callback(self, data):
        # Extract beam directions, range bin values, and image data from sonar_image_raw
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.ping_info = data.ping_info
        # Convert the raw data into a numpy array
        if self.beam_directions and self.ranges and data.image.data and self.ping_info:
            # ADAPTED: decode FLOAT32 (DAVE) or UINT8 to the original UINT8 scale
            self.data_raw = sonar_image_uint8(data, self.sonar_float_gain)
            # ADAPTED: the original assumed exactly 512 beams; DAVE sends 513
            self.beam_count = data.image.beam_count or len(self.beam_directions)
            self.data_available = True

    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """
        Convert polar coordinates (i, j) to Cartesian coordinates.
        The image appears rotated by 90 degrees, so adjust accordingly.
        """
        # here max distance is assigned 15
        angle_per_beam = 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi / 4
        distance = j * 15 / max_bins
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y

    def print_polynomial_coefficients(self, coeffs):
        print('Polynomial Coefficients:')
        print(f'y = {coeffs[0]:.4f}x^2 + {coeffs[1]:.4f}x + {coeffs[2]:.4f}')

    def compute_polynomial_derivative(self, coeffs):
        a, b, c = coeffs
        return [2 * a, b]

    def find_and_plot_curve(self, x_coords, y_coords):
        # Ensure x_coords are strictly increasing
        sorted_points = sorted(zip(x_coords, y_coords))
        x_coords, y_coords = zip(*sorted_points)
        self.get_logger().info('plotter')
        # Check if x_coords is strictly increasing
        if any(x2 <= x1 for x1, x2 in zip(x_coords, x_coords[1:])):
            self.get_logger().warning('x_coords are not strictly increasing. Removing duplicates.')
            unique_points = []
            for x, y in zip(x_coords, y_coords):
                if not unique_points or unique_points[-1][0] != x:
                    unique_points.append((x, y))
            x_coords, y_coords = zip(*unique_points)

        # Perform polynomial interpolation (2nd degree)
        coeffs = np.polyfit(x_coords, y_coords, 2)
        self.print_polynomial_coefficients(coeffs)

        # Compute derivative coefficients
        derivative_coeffs = self.compute_polynomial_derivative(coeffs)
        self.a_poly = derivative_coeffs[0] / 2

        # Calculate slopes at unique contour points
        self.slopes = []
        positive_tangent_slopes = []
        negative_tangent_slopes = []

        for x, y in zip(x_coords, y_coords):
            tangent_slope = np.polyval(derivative_coeffs, x)
            if x > 0:
                positive_tangent_slopes.append(tangent_slope)
            elif x < 0:
                negative_tangent_slopes.append(tangent_slope)

        self.slopes = positive_tangent_slopes + negative_tangent_slopes
        # Compute average slopes
        avg_positive_slope = np.mean(positive_tangent_slopes) if positive_tangent_slopes else 0
        avg_negative_slope = np.mean(negative_tangent_slopes) if negative_tangent_slopes else 0
        self.avg_right_slope = avg_positive_slope
        self.avg_left_slope = avg_negative_slope
        # Compare magnitudes
        magnitude_positive = abs(avg_positive_slope)
        magnitude_negative = abs(avg_negative_slope)

        if magnitude_positive > magnitude_negative:
            dominant_slope = avg_positive_slope
        else:
            dominant_slope = avg_negative_slope

    def process_data(self):
        self.orient_3d = False
        if not self.turn_around:
            beam_count = self.beam_count  # ADAPTED: was 512
            range_count = len(self.ranges)
            threshold = 2

            self.data = np.array(self.data_raw).reshape((range_count, beam_count))

            obstacle_free_beam_numbers = []
            self.criti_cal = False
            range_window = 3

            contour_points = []
            # Iterate over the range of beam indices
            for i in range(0, beam_count, 5):
                critical_obstacle_detected = False
                # Iterate over the range of range bin indices
                for j in range(10, range_count - 40, 5):
                    # Extract the window of intensities
                    window_intensities = self.data[j:j + range_window, i]

                    # Compute the average intensity
                    window_average_intensity = np.mean(window_intensities)

                    # Check against the thresholdbreak
                    if window_average_intensity > threshold:
                        contour_points.append((i, j))
                        critical_obstacle_detected = True
                        break  # Exit the loop if the condition is met

                if not critical_obstacle_detected:
                    obstacle_free_beam_numbers.append(i)

            is_covered = False

            if obstacle_free_beam_numbers:
                is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)

            if is_covered:
                self.publish_heading(go_beam_no, 1)

            else:
                print('no gap')
                boundedness_number = self.check_for_boundedness(obstacle_free_beam_numbers)

                if boundedness_number == 1 and self.right_goal:
                    print('right bounded')
                    self.publish_heading(10, 1)
                elif boundedness_number == 1 and self.left_goal:
                    print('left bounded')
                    self.publish_heading(500, 1)
                elif boundedness_number == 2 and self.left_goal:
                    print('turn right')
                    self.publish_heading(10, 1)
                elif boundedness_number == 2 and self.right_goal:
                    print('turn left')
                    self.publish_heading(500, 1)
                else:
                    print('contour_1')

                    points_cartesian = [
                        self.polar_to_cartesian(i, j, beam_count, range_count)
                        for (i, j) in contour_points]

                    if points_cartesian:  # Extract x and y coordinates
                        x_coords, y_coords = zip(*points_cartesian)

                        self.a_poly = None
                        # Find and plot the curve
                        self.find_and_plot_curve(x_coords, y_coords)

                        print(self.a_poly)

                    # ADAPTED: in Python 3 `None < 0.02` raises; the original
                    # only reached it with no contour ever fitted.
                    if self.a_poly is not None and self.a_poly < 0.02:

                        if not self.xy_called:
                            print('3d')
                            print(self.a_poly)
                            self.check_verti_go = True
                            self.navigate_3d(True)

                        else:
                            self.publish_heading(self.avg_left_slope, 2)  # this might not be be latest since i am rotating rover at 3d

                    else:
                        print('3d not required')
                        if self.left_goal:
                            # check contour slope
                            self.publish_heading(self.avg_left_slope, 2)

                        elif self.right_goal:
                            # check contour slope
                            self.publish_heading(self.avg_right_slope, 2)

        if self.turn_around:
            self.publish_heading(256, False)

    def orient_for_3d(self):
        # Calculate the global angle in degrees to beam number
        def global_angle_to_beam_number(global_angle):
            # Convert angle to beam number
            total_beams = 512
            angle_range_start = 0.7853  # Starting angle of the beam range
            angle_range_end = 2.35619
            rad_per_beam = 1.57 / total_beams

            if angle_range_start <= global_angle <= angle_range_end:  # convert the angle range to radians or will give error
                return int((global_angle - angle_range_start) / rad_per_beam)
            elif global_angle > 2.35619:
                return 512  # Global angle out of the beam range, add logic for side beams, ie (0,45) and (135,180)
            elif global_angle < 0.7853:
                return 0

        if 1.4 < self.global_angle < 1.7:
            self.orient_3d = True
            print('3d - global_angle')
            print(self.global_angle)
            self.navigate_3d(False)

        else:
            print('orienting 3d global angle')
            self.orient_3d = False
            go_to_beam = global_angle_to_beam_number(self.global_angle)
            print(go_to_beam)
            self.publish_heading(go_to_beam, 3)

    def check_for_boundedness(self, obstacle_free_beam_numbers):
        self.right_goal = False
        self.left_goal = False
        print(obstacle_free_beam_numbers)
        if not self.turn_around:

            # Check if the angle is between 1.5 and 3.14
            if 1.5 <= self.global_angle <= 3.14:
                self.right_goal = True
                self.left_goal = False
            # Check if the angle is between 0 and 1.5
            elif 0 <= self.global_angle < 1.5:
                self.left_goal = True
                self.right_goal = False

            beams_to_check_1 = {0, 5, 10, 15, 20, 25, 30, 35}
            beams_to_check_2 = {510, 505, 500, 495, 490, 485, 480}

            # Check if beam 0 is present
            has_beam_0 = beams_to_check_1.issubset(obstacle_free_beam_numbers)
            # Check if beam 512 is present
            has_beam_512 = beams_to_check_2.issubset(obstacle_free_beam_numbers)

            # Determine the return value based on the presence of the beams
            if has_beam_0 and self.right_goal:
                return 1
            elif has_beam_512 and self.left_goal:
                return 1
            elif has_beam_0 and self.left_goal:
                return 2
            elif has_beam_512 and self.right_goal:
                return 2

            else:
                print('detected object not bounded')
                return 4

    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        # Constants
        total_beams = 512
        angle_per_beam = 90.0 / total_beams
        angle_range_start = 0.7853  # Starting angle of the beam range
        angle_range_end = 2.35619
        rad_per_beam = 1.57 / total_beams

        required_angle = 15.0

        # Calculate the number of beams needed to cover 10 degrees
        required_beams = int(required_angle / (angle_per_beam * 5))

        # Calculate the global angle in degrees to beam number
        def global_angle_to_beam_number(global_angle):
            # Convert angle to beam number
            if angle_range_start <= global_angle <= angle_range_end:  # convert the angle range to radians or will give error
                return int((global_angle - angle_range_start) / rad_per_beam)
            elif global_angle > 2.35619:
                return 512  # Global angle out of the beam range, add logic for side beams, ie (0,45) and (135,180)
            elif global_angle < 0.7853:
                return 0

        # Sort the beam numbers
        obstacle_free_beam_numbers.sort()

        # Check for continuous beams
        mid_beam = []
        for i in range(len(obstacle_free_beam_numbers) - required_beams):
            if obstacle_free_beam_numbers[i + required_beams] - obstacle_free_beam_numbers[i] == 5 * required_beams:
                mid_index = i + required_beams // 2
                mid_beam.append(obstacle_free_beam_numbers[mid_index])

        target_beam_number = global_angle_to_beam_number(self.global_angle)

        self.target_beam = target_beam_number

        if mid_beam:
            # Find the beam closest to the global angle
            closest_to_target = min(mid_beam, key=lambda x: abs(x - target_beam_number))

            return True, closest_to_target

        return False, None

    def publish_heading(self, beam_number, move):
        if not self.turn_around:
            # Assume the total angle formed by all beams is 90 degrees
            if move == 1 or move == 3:
                total_angle_degrees = 90.0
                # Calculate the angle formed by each beam
                num_beams = 512
                angle_per_beam_degrees = total_angle_degrees / num_beams

                # Calculate the desired heading in degrees based on the beam number with 0 as center beam-> to get -ve velocity
                desired_heading_degrees = (beam_number - (num_beams // 2)) * angle_per_beam_degrees

                # Convert the desired heading to radians
                desired_heading_radians = math.radians(desired_heading_degrees)

                Kp = 0.12  # Proportional gain (tuning parameter)
                Kv = 0.35

                # Calculate the angular velocity
                angular_velocity = Kp * desired_heading_radians

                linear_velocity_y = 0
                linear_velocity_x = 0
                if move == 1:
                    linear_velocity_x = Kv * (1.6 - abs(desired_heading_radians))

            elif move == 2:
                desired_heading_radians = beam_number  # here beam number is desired hading value in slope directly from the call
                angular_velocity = 0.1 * desired_heading_radians
                linear_velocity_x = 0.0
                linear_velocity_y = 0.0
                print(f'entered contour based navigation with desired slope: {self.dominant_slope}, '
                      f'Vx: {linear_velocity_x}, linear velocity: {linear_velocity_y}')

        if self.turn_around:
            angular_velocity = 1.0
            linear_velocity_x = 0.0
            linear_velocity_y = 0.0

        # Create the Twist message
        twist = Twist()
        twist.angular.z = float(angular_velocity)  # Set the angular velocity
        twist.linear.x = float(linear_velocity_x)
        twist.linear.y = float(linear_velocity_y)
        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)

    def avoidance(self):
        if not self.in_z_bound:
            self.check_verti_go = True
            self.navigate_3d(False)

        if self.in_z_bound:
            self.process_data()

    def move_sonar(self):
        current_time = self.get_time() - self.start_3d_time
        cycle_duration = 0.5 / self.frequency
        if current_time < cycle_duration:
            # Collect data if within the current cycle

            if self.ttd:
                # Compute the sine wave position
                z_position = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time + math.pi / 2) + self.offset
                degree_angle = int(z_position * (180 / math.pi))

                # Round to the closest multiple of 3
                self.sonar_angle = round(degree_angle / 3) * 3  # converting to in degree multiple of 3

                # Publish the current joint angle
                self.publish_float(self.joint_pub, z_position)
                self.process_3d_data(self.sonar_angle)

            elif not self.ttd:
                # Compute the sine wave position
                z_position = self.amplitude * math.sin(
                    2 * math.pi * self.frequency * (current_time + cycle_duration) + math.pi / 2) + self.offset
                degree_angle = int(z_position * (180 / math.pi))

                # Round to the closest multiple of 3
                self.sonar_angle = round(degree_angle / 3) * 3  # converting to in degree multiple of 3

                # Publish the current joint angle
                self.publish_float(self.joint_pub, z_position)
                self.process_3d_data(self.sonar_angle)

        else:
            self.latest_scan_available = True

            self.ttd = not self.ttd
            if self.min_ranges:
                self.min_range = min(self.min_ranges) - 2
            else:
                self.min_range = 7
            self.min_ranges = []
            print('completed a full cycle.')

            self.pose_before_z_motion = self.pose  # starting pose of vehicle before executing the z angle motion
            self.navigate_3d(False)

    def process_3d_data(self, sonar_angle):
        beam_count = self.beam_count  # ADAPTED: was 512
        range_count = len(self.ranges)
        threshold = 15

        self.data = np.array(self.data_raw).reshape((range_count, beam_count))

        data = self.data

        break_outer_loop = False

        for j in range(5, 350, 5):
            if break_outer_loop:
                break
            # iterate thru beam
            for i in range(100, 400, 7):
                # Check against the threshold
                if data[j, i] > threshold:
                    print('exced threshold')
                    self.min_ranges.append(j * (12 / len(self.ranges)))
                    break_outer_loop = True

                    break  # Exit the inner loop

        if not break_outer_loop:
            self.go_vertical_angle.append(int(sonar_angle))

    def move_sonar_extreme(self):
        self.publish_float(self.joint_pub, 0.8)

    def start_data_collection(self):
        self.collect_3d_data_flag = True
        self.start_3d_time = self.get_time()

    def stop_data_collection(self):
        print('stop_data_collection')
        self.collect_3d_data_flag = False
        self.publish_float(self.joint_pub, 0)
        self.publish_float(self.joint_pub, 0)
        self.sonar_centered = True

    def navigate_3d(self, xy_called):
        if self.latest_scan_available:
            self.latest_scan_available = False
            mid_beam = []
            required_beams = 5

            obstacle_free_beam_numbers = self.go_vertical_angle
            print(f'go_vertical_angle {self.go_vertical_angle}')
            self.go_vertical_angle = []
            if obstacle_free_beam_numbers:

                array = np.array(obstacle_free_beam_numbers)

                # Use np.unique to sort and remove duplicates
                unique_sorted_array = np.unique(array)

                # Convert back to a list if needed
                obstacle_free_beam_numbers = unique_sorted_array.tolist()

                for i in range(len(obstacle_free_beam_numbers) - required_beams):
                    if obstacle_free_beam_numbers[i + required_beams] - obstacle_free_beam_numbers[i] == 3 * required_beams:
                        mid_index = i + required_beams // 2
                        mid_beam.append(obstacle_free_beam_numbers[mid_index])

            if mid_beam:
                # Find the beam closest to the global angle
                self.closest_to_target_vertical = min(mid_beam, key=lambda x: abs(x - self.angle_z_goal))
                self.new_midb_available = True
                self.z_motion_ongoing = True
                self.start_data_collection()

            else:
                self.z_motion_ongoing = False
                self.stop_data_collection()
                if xy_called:
                    self.xy_called = True

                    print(' going back to process data')

                else:
                    self.xy_called = False

                    print('stop_data_collection xy not called')
                    self.process_data  # as in the original: a no-op reference, not a call
        else:
            self.collect_3d_data_flag = True
            if self.orient_3d:
                self.start_data_collection()
                if self.sonar_centered:
                    self.move_sonar_extreme()

    def move_in_z(self):
        if self.closest_to_target_vertical:
            twist = Twist()
            if self.closest_to_target_vertical == 0:
                self.closest_to_target_vertical = 1
            if math.sin(self.closest_to_target_vertical) == 0:
                raise ValueError('Angle cannot be 0 degrees or 180 degrees as it leads to division by zero.')

            linear_x = 0.5
            linear_z = linear_x * math.tan(self.closest_to_target_vertical * (math.pi / 180))
            print(f'losest_to_target_vertical{self.closest_to_target_vertical}')
            twist = Twist()

            # Set the linear velocities
            twist.linear.x = float(linear_x)
            twist.linear.z = float(linear_z)
            self.cmd_vel_pub.publish(twist)

            # ADAPTED: re-enabled the original's commented-out exit (distance
            # travelled since the last sweep > min_range, i.e. nearest
            # obstacle seen in the sweep - 2 m, or 7 m if none). Without it
            # the z motion only ends when a sweep finds no vertical gap, which
            # never happens in open water, so the vehicle climbed away in a
            # straight line indefinitely. stop_data_collection() is added so
            # the sonar recentres and 2D navigation resumes.
            if self.pose_before_z_motion is not None and \
                    self.compute_distance(self.pose_before_z_motion, self.pose) > self.min_range:
                self.go_vertical_angle = []
                self.z_motion_ongoing = False
                self.stop_data_collection()

    def compute_distance(self, pose1, pose2):
        pos1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        pos2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        return np.linalg.norm(pos2 - pos1)

    def run_once(self):
        if self.rospy_first_time:
            # ADAPTED: the original waited for /gazebo/spawn_sdf_model and spawned
            # a yellow marker ball (Gazebo Classic only; visual, not used by
            # the algorithm), then centred the sonar.
            print('1st time')
            self.stop_data_collection()
            self.rospy_first_time = False
        # ADAPTED: also wait for the first pose (move_sonar reads it); in ROS 1
        # pose_gt was already flowing by the time the first sonar frame came.
        if self.data_available and self.pose is not None:
            if self.z_motion_ongoing:

                print('z motion ongoing')
                self.publish_float(self.sonar_move_pub, 1)
                self.move_in_z()
                if self.collect_3d_data_flag:
                    self.move_sonar()

            else:
                if self.collect_3d_data_flag:
                    if self.orient_3d:
                        self.publish_float(self.sonar_move_pub, 2)
                        self.move_sonar()
                    else:
                        self.publish_float(self.sonar_move_pub, 0)
                        self.orient_for_3d()
                else:
                    self.publish_float(self.sonar_move_pub, 0)
                    self.avoidance()


def main():
    rclpy.init()
    node = SonarHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
