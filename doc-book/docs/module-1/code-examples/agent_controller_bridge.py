#!/usr/bin/env python3

"""
Agent-Controller Bridge Pattern Example
This code demonstrates the bridge pattern between high-level AI logic
and low-level ROS controllers as covered in Chapter 2 of the textbook.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import random
import math
from enum import Enum


class RobotState(Enum):
    """Enumeration for robot states"""
    IDLE = "idle"
    EXPLORING = "exploring"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    RETURNING_HOME = "returning_home"
    CHARGING = "charging"


class AgentLayer(Node):
    """
    The Agent Layer - High-level decision making
    This represents the 'mind' of the robot that makes decisions based on
    sensor data and goals.
    """
    def __init__(self):
        super().__init__('agent_layer')

        # Subscribe to sensor data from the controller layer
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'sensor_scan',
            self.laser_callback,
            10
        )

        self.battery_subscriber = self.create_subscription(
            Float32,
            'battery_level',
            self.battery_callback,
            10
        )

        # Publish commands to the controller layer
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for decision making
        self.decision_timer = self.create_timer(0.1, self.decision_callback)

        # Internal state
        self.safety_distance = 0.8  # meters
        self.low_battery_threshold = 0.2  # 20%
        self.battery_level = 1.0
        self.obstacle_distances = []
        self.current_state = RobotState.EXPLORING
        self.home_position = (0.0, 0.0)  # Starting position

    def laser_callback(self, msg):
        """Process laser scan data from controller"""
        if len(msg.ranges) > 0:
            # Store obstacle distances for analysis
            self.obstacle_distances = [r for r in msg.ranges if 0 < r < msg.range_max]

            # Check for immediate obstacles
            if self.obstacle_distances:
                min_distance = min(self.obstacle_distances)
                if min_distance < self.safety_distance:
                    if self.current_state != RobotState.AVOIDING_OBSTACLE:
                        self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m, switching to avoidance')
                    self.current_state = RobotState.AVOIDING_OBSTACLE
                elif self.current_state == RobotState.AVOIDING_OBSTACLE:
                    # If we were avoiding obstacles and now they're clear, return to exploration
                    self.current_state = RobotState.EXPLORING

    def battery_callback(self, msg):
        """Process battery data from controller"""
        self.battery_level = msg.data
        if self.battery_level < self.low_battery_threshold:
            if self.current_state != RobotState.RETURNING_HOME:
                self.get_logger().info(f'Battery low ({self.battery_level*100:.1f}%), returning home')
            self.current_state = RobotState.RETURNING_HOME

    def decision_callback(self):
        """Make high-level decisions based on current state"""
        cmd_msg = Twist()

        if self.current_state == RobotState.RETURNING_HOME:
            # Simple return home behavior (in a real system, this would use path planning)
            cmd_msg.linear.x = -0.2  # Move backward toward home
            cmd_msg.angular.z = 0.0
            self.get_logger().info('Returning to charging station')
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            # Implement obstacle avoidance behavior
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.6  # Turn to avoid obstacle
            self.get_logger().info('Avoiding obstacle')
        elif self.current_state == RobotState.EXPLORING:
            # Random walk exploration
            cmd_msg.linear.x = 0.4
            cmd_msg.angular.z = random.uniform(-0.3, 0.3)
            self.get_logger().info('Exploring environment')
        else:
            # Default: stop
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        # Publish the command to the controller layer
        self.cmd_publisher.publish(cmd_msg)


class ControllerLayer(Node):
    """
    The Controller Layer - Low-level command execution
    This represents the 'body' of the robot that executes commands
    and provides sensor feedback.
    """
    def __init__(self):
        super().__init__('controller_layer')

        # Subscribe to commands from the agent layer
        self.cmd_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        # Publish sensor data to the agent layer
        self.laser_publisher = self.create_publisher(
            LaserScan,
            'sensor_scan',
            10
        )

        self.battery_publisher = self.create_publisher(
            Float32,
            'battery_level',
            10
        )

        # Timer to simulate sensor data publishing
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)

        # Timer to simulate battery drain
        self.battery_timer = self.create_timer(5.0, self.publish_battery_data)

        # Internal state for simulation
        self.current_cmd = Twist()
        self.battery_level = 1.0  # Start fully charged
        self.simulated_laser_data = self.generate_laser_scan()

    def cmd_callback(self, msg):
        """Receive commands from the agent layer"""
        self.current_cmd = msg
        self.get_logger().info(f'Controller received command: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def generate_laser_scan(self):
        """Generate simulated laser scan data"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 180  # 1 degree increments
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = 0.1
        scan.range_max = 10.0

        # Generate 180 points (from -90 to +90 degrees)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = []

        for i in range(num_readings):
            # Simulate various distances with some random obstacles
            angle = scan.angle_min + i * scan.angle_increment
            distance = 3.0  # Default distance

            # Add some simulated obstacles
            if 0.5 < abs(angle) < 0.7:  # Obstacle in front-left or front-right
                distance = 0.8
            elif abs(angle) < 0.2:  # Clear path ahead
                distance = 5.0
            elif 1.2 < abs(angle) < 1.4:  # Obstacle to the side
                distance = 1.2

            scan.ranges.append(distance)

        return scan

    def publish_sensor_data(self):
        """Publish simulated sensor data to the agent layer"""
        # Update the timestamp
        self.simulated_laser_data.header.stamp = self.get_clock().now().to_msg()

        # Add some random variation to simulate real sensor data
        for i in range(len(self.simulated_laser_data.ranges)):
            # Add small random variation to each reading
            original_distance = self.simulated_laser_data.ranges[i]
            if original_distance < self.simulated_laser_data.range_max:
                variation = random.uniform(-0.05, 0.05)
                new_distance = max(self.simulated_laser_data.range_min,
                                 min(original_distance + variation, self.simulated_laser_data.range_max))
                self.simulated_laser_data.ranges[i] = new_distance

        self.laser_publisher.publish(self.simulated_laser_data)

    def publish_battery_data(self):
        """Publish battery level with simulated drain"""
        # Simulate battery drain based on activity
        drain_rate = 0.001  # Per battery publication (every 5 seconds)
        self.battery_level = max(0.0, self.battery_level - drain_rate)

        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_publisher.publish(battery_msg)

        self.get_logger().info(f'Battery level: {self.battery_level*100:.1f}%')


def main(args=None):
    """Main function to run both agent and controller layers"""
    rclpy.init(args=args)

    # Create both nodes
    agent = AgentLayer()
    controller = ControllerLayer()

    try:
        # Create an executor that handles both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(agent)
        executor.add_node(controller)

        # Spin both nodes
        agent.get_logger().info('Agent-Controller Bridge initialized. Starting execution...')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        agent.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()