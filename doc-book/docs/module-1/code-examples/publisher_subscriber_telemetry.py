#!/usr/bin/env python3

"""
Simple publisher/subscriber example for robot telemetry
This code demonstrates the basic ROS 2 communication concepts
covered in Chapter 1 of the textbook.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import math


class TelemetryPublisher(Node):
    """
    A simple publisher that sends robot telemetry data
    """
    def __init__(self):
        super().__init__('telemetry_publisher')

        # Create publisher for telemetry data
        self.publisher_ = self.create_publisher(String, 'robot_telemetry', 10)

        # Create a timer to publish data periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        """Callback function that publishes telemetry data"""
        msg = String()

        # Generate simulated telemetry data
        battery_level = 100.0 - (self.i * 0.1)  # Simulate battery drain
        temperature = 25.0 + random.uniform(-2, 2)  # Simulate temperature variation
        position_x = math.sin(self.i * 0.1) * 10  # Simulate movement
        position_y = math.cos(self.i * 0.1) * 5  # Simulate movement

        # Format the telemetry message
        msg.data = f'Robot status: Operational | Battery: {battery_level:.1f}% | ' \
                   f'Temp: {temperature:.1f}°C | Pos: ({position_x:.2f}, {position_y:.2f}) | ' \
                   f'Msg #{self.i}'

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class TelemetrySubscriber(Node):
    """
    A simple subscriber that receives and processes telemetry data
    """
    def __init__(self):
        super().__init__('telemetry_subscriber')

        # Create subscription to telemetry topic
        self.subscription = self.create_subscription(
            String,
            'robot_telemetry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """Callback function that processes received telemetry data"""
        self.get_logger().info(f'Received telemetry: "{msg.data}"')

        # Extract information from the message (simplified parsing)
        if 'Battery:' in msg.data:
            try:
                # Extract battery percentage
                start_idx = msg.data.find('Battery: ') + len('Battery: ')
                end_idx = msg.data.find('%', start_idx)
                battery_str = msg.data[start_idx:end_idx]
                battery_level = float(battery_str)

                # Log warning if battery is low
                if battery_level < 20.0:
                    self.get_logger().warn(f'Low battery warning: {battery_level}%')
            except (ValueError, IndexError):
                self.get_logger().info('Could not parse battery level from message')


def main(args=None):
    """Main function to run the publisher and subscriber nodes"""
    rclpy.init(args=args)

    # Create both nodes
    telemetry_publisher = TelemetryPublisher()
    telemetry_subscriber = TelemetrySubscriber()

    try:
        # Create an executor that handles both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(telemetry_publisher)
        executor.add_node(telemetry_subscriber)

        # Spin both nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        telemetry_publisher.destroy_node()
        telemetry_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()