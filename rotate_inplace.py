#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
import math
import time
from vincenty import vincenty

class SquareWaypoints(Node):
    def __init__(self):
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          depth=1)

        super().__init__('square_waypoints')

        # Initialize waypoints
        self.waypoints = [
            (23.28884098, 77.27393332), (23.28883891, 77.27402810), (23.28884098, 77.27412853), (23.28884305, 77.27422443),
            (23.28893114, 77.27422443), (23.28893218, 77.27412740), (23.28893218, 77.27402810), (23.28893114, 77.27393107),
            (23.28901924, 77.27393220), (23.28901924, 77.27402810), (23.28901820, 77.27412853), (23.28901924, 77.27422330)
        ]

        self.current_waypoint_index = 0
        self.sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.callback, self.qos_policy)
        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

    def callback(self, msg):
        self.get_logger().info('Current position: {}, {}'.format(msg.pose.position.x, msg.pose.position.y))

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = msg.pose.position.x
        pose_stamped.pose.position.y = msg.pose.position.y 
        pose_stamped.pose.position.z = msg.pose.position.z

        # Calculate yaw for 360-degree rotation
        num_steps = 8  # Number of steps for a full rotation
        step_angle = 360.0 / num_steps  # Angle to rotate at each step
        for step in range(num_steps):
            yaw_degrees = step * step_angle  # Calculate yaw for the current step
            pose_stamped.pose.orientation = self.calculate_orientation(yaw_degrees)  # Set the orientation

            # Publish the pose_stamped message to control the drone's orientation
            self.pub.publish(pose_stamped)
            print("test")
            time.sleep(35)


    def calculate_orientation(self, yaw_degrees):
        # You need to implement a function to convert yaw in degrees to a Quaternion
        # This depends on the frame convention used in your specific setup
        # Here's a placeholder function that assumes the yaw is converted directly to a Quaternion
        # You may need to adjust this function based on your actual needs.

        # Convert degrees to radians
        yaw_radians = math.radians(yaw_degrees)
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw_radians / 2)
        quaternion.w = math.cos(yaw_radians / 2)

        return quaternion

def main(args=None):
    rclpy.init()
    node = SquareWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()