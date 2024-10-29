import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription 

        self.prev_ranges = []  # Store previous laser readings
        self.prev_angles = []  # Store previous angles for smoothing
        self.min_distance = 0.4  # Set a minimum distance to avoid collisions
        self.angle_filter_window = 5  # Window size for angle smoothing

    def detect_person(self, ranges):
        # Logic to determine if there is a person in the laser data
        min_range = min(ranges)
        person_threshold = 1.5  # Threshold for person detection
        obstacle_threshold = 0.25  # Threshold for obstacle detection
        
        # Check if the minimum range is below the obstacle threshold
        if min_range < obstacle_threshold:
            return False  # If there's an obstacle too close, don't consider it as a person
        
        # Check if the minimum range is below the person threshold
        if min_range < person_threshold:
            return True  # If the range is below the person threshold, consider it as a person
        
        # If neither obstacle nor person is detected, return False
        return False

    def smooth_angle(self, angle):
        # Add the new angle to the list of previous angles
        self.prev_angles.append(angle)

        # If the list exceeds the size of the window, remove the oldest angle
        if len(self.prev_angles) > self.angle_filter_window:
            self.prev_angles.pop(0)

        # Calculate the average angle from the angles stored in the list
        avg_angle = sum(self.prev_angles) / len(self.prev_angles)
        
        return avg_angle

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges

        # Check if a person is detected
        if self.detect_person(ranges):
            # Find the index of the minimum range (closest obstacle)
            min_range_index = ranges.index(min(ranges))

            # Calculate the angle to the person
            angle_to_person = angle_min + min_range_index * angle_increment

            # Smooth out the angle change
            if self.prev_angles:
                angle_to_person = self.smooth_angle(angle_to_person)

            # Check if the robot is too close to the person
            if min(ranges) < self.min_distance:
                # Reduce the forward velocity
                vx = 0.05  # Reduced velocity
            else:
                vx = 0.35  # Normal velocity

            # Use the angle to the person as the target angle for the robot to aim towards
            target_angle = angle_to_person

            # Calculate the difference between the target angle and the current angle (heading) of the robot
            angle_difference = -target_angle  # Negative to change the orientation to move

            # Limit the angular velocity to avoid excessive spinning
            max_angular_velocity = 0.6
            if abs(angle_difference) > max_angular_velocity:
                angle_difference = max_angular_velocity if angle_difference > 0 else -max_angular_velocity

            # Set the angular velocity of the robot based on the difference in angles
            wz = 2.0 * angle_difference

            # Create Twist message and publish
            output_msg = Twist()
            output_msg.linear.x = vx
            output_msg.angular.z = wz
            self.publisher_.publish(output_msg)
        else:
            # If no person is detected, stop the robot
            vx = 0.0
            wz = 0.0

            # Create Twist message and publish
            output_msg = Twist()
            output_msg.linear.x = vx
            output_msg.angular.z = wz
            self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
