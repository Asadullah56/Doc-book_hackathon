---
title: "Chapter 1 - The ROS 2 Communication Backbone"
sidebar_position: 1
description: "Understanding the fundamental concepts of ROS 2 communication: Nodes, Topics, Services, and Actions"
---

import TranslationPersonalizationBar from '@site/src/components/TranslationPersonalizationBar';

# Chapter 1: The ROS 2 Communication Backbone

<TranslationPersonalizationBar />

## Introduction

Welcome to the foundational chapter of our Physical AI and Humanoid Robotics textbook. In this chapter, we'll explore the core concepts of ROS 2 (Robot Operating System 2) communication architecture. Understanding how different components of a robotic system communicate with each other is crucial for building complex robotic applications.

ROS 2 provides a flexible framework for distributed computing, allowing multiple processes to communicate with each other in a structured way. This communication infrastructure forms the "nervous system" of a robot, enabling different parts of the system to coordinate and work together effectively.

## 1. Understanding Nodes

A **Node** is the fundamental unit of computation in ROS 2. It's an executable process that performs specific tasks and communicates with other nodes. Think of nodes as specialized workers in a factory, each responsible for a particular function.

### Creating Your First Node

Let's create a simple node that publishes robot telemetry data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_telemetry', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status: Operational - Message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    telemetry_publisher = TelemetryPublisher()

    try:
        rclpy.spin(telemetry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**APA Citation**: Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/

## 2. Topics and Message Passing

**Topics** enable asynchronous communication between nodes using a publish-subscribe model. A publisher node sends messages to a topic, and subscriber nodes receive those messages. This decouples the publisher from the subscriber, allowing for flexible system design.

### Creating a Subscriber Node

Here's a subscriber that receives the telemetry messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_telemetry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    telemetry_subscriber = TelemetrySubscriber()

    try:
        rclpy.spin(telemetry_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Services: Synchronous Communication

While topics provide asynchronous communication, **Services** enable synchronous request-response communication. A client sends a request to a service, and the service processes the request and returns a response.

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}\n')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Actions: Advanced Communication Pattern

**Actions** are used for long-running tasks that provide feedback during execution. They combine the features of services and topics, allowing for goal requests, feedback during execution, and final results.

## 5. The Communication Graph

ROS 2 systems form a communication graph where nodes are connected through topics, services, and actions. This graph can be visualized using tools like `rqt_graph`, which shows the relationships between nodes and the topics they communicate through.

Understanding this graph is essential for debugging and optimizing robotic systems. It helps identify bottlenecks, visualize data flow, and understand the dependencies between different components.

## Summary

In this chapter, we've covered the fundamental communication concepts in ROS 2:

- **Nodes**: The basic computational units
- **Topics**: Asynchronous publish-subscribe communication
- **Services**: Synchronous request-response communication
- **Actions**: Advanced communication for long-running tasks

These concepts form the backbone of ROS 2 communication and are essential for building distributed robotic systems. In the next chapter, we'll explore how to bridge Python-based AI logic with these ROS 2 communication patterns using the rclpy library.

## Exercises

1. Modify the publisher example to publish different types of robot status messages
2. Create a subscriber that processes and filters the telemetry messages
3. Implement a service that provides robot status information on request