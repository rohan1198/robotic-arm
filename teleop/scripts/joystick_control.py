#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import numpy as np

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('linear_x_axis', 1)  # Left stick Y-axis
        self.declare_parameter('linear_y_axis', 0)  # Left stick X-axis
        self.declare_parameter('angular_z_axis', 2)  # Right stick X-axis
        self.declare_parameter('joint_4_axis', 3)  # Right stick Y-axis
        self.declare_parameter('joint_5_axis', 6)  # D-pad X-axis
        self.declare_parameter('joint_6_axis', 7)  # D-pad Y-axis
        self.declare_parameter('deadman_button', 4)  # L1 button
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 0.5)
        self.declare_parameter('scale_joint', 0.5)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.linear_x_axis = self.get_parameter('linear_x_axis').value
        self.linear_y_axis = self.get_parameter('linear_y_axis').value
        self.angular_z_axis = self.get_parameter('angular_z_axis').value
        self.joint_4_axis = self.get_parameter('joint_4_axis').value
        self.joint_5_axis = self.get_parameter('joint_5_axis').value
        self.joint_6_axis = self.get_parameter('joint_6_axis').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.scale_joint = self.get_parameter('scale_joint').value
        
        # Initialize joint names and positions
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Create publishers and subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/robotic_arm_controller/joint_trajectory',
            10)
        
        # Create timer for publishing trajectory
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_trajectory)
        
        self.deadman_pressed = False
        
        self.get_logger().info('Joystick control node initialized')
        self.get_logger().info('Press L1 to enable control')
    
    def joy_callback(self, msg):
        # Check deadman button (L1)
        if len(msg.buttons) > self.deadman_button:
            self.deadman_pressed = msg.buttons[self.deadman_button] == 1
        
        if not self.deadman_pressed:
            return
        
        # Map joystick axes to joint movements
        if len(msg.axes) > self.linear_y_axis:
            # Joint 1 - Base rotation (Left stick X)
            self.joint_positions[0] += msg.axes[self.linear_y_axis] * self.scale_linear * 0.05
        
        if len(msg.axes) > self.linear_x_axis:
            # Joint 2 - Shoulder (Left stick Y)
            self.joint_positions[1] += msg.axes[self.linear_x_axis] * self.scale_linear * 0.05
        
        if len(msg.axes) > self.angular_z_axis:
            # Joint 3 - Elbow (Right stick X)
            self.joint_positions[2] += msg.axes[self.angular_z_axis] * self.scale_angular * 0.05
        
        if len(msg.axes) > self.joint_4_axis:
            # Joint 4 - Wrist roll (Right stick Y)
            self.joint_positions[3] += msg.axes[self.joint_4_axis] * self.scale_joint * 0.05
        
        if len(msg.axes) > self.joint_5_axis:
            # Joint 5 - Wrist pitch (D-pad X)
            self.joint_positions[4] += msg.axes[self.joint_5_axis] * self.scale_joint * 0.05
        
        if len(msg.axes) > self.joint_6_axis:
            # Joint 6 - Wrist yaw (D-pad Y)
            self.joint_positions[5] += msg.axes[self.joint_6_axis] * self.scale_joint * 0.05
        
        # Apply joint limits (consider adding proper limits for each joint)
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = max(-3.14, min(3.14, self.joint_positions[i]))
    
    def publish_trajectory(self):
        if not self.deadman_pressed:
            return
        
        # Create trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set trajectory point
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0.0] * 6  # Zero velocity at target
        point.time_from_start.sec = 1  # Reach target in 1 second
        
        trajectory_msg.points = [point]
        
        # Publish trajectory
        self.traj_pub.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    
    try:
        rclpy.spin(joystick_control)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
