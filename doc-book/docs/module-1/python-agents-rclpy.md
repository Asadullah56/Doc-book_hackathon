---
title: "Chapter 2 - Programming the Robot Mind: Python Agents with rclpy"
sidebar_position: 2
description: "Bridging Python AI logic to low-level ROS controllers using rclpy and implementing the Agent-Controller bridge pattern"
---

import TranslationPersonalizationBar from '@site/src/components/TranslationPersonalizationBar';

# Chapter 2: Programming the Robot Mind - Python Agents with rclpy

<TranslationPersonalizationBar />

## Introduction

In the previous chapter, we explored the communication backbone of ROS 2. Now we'll dive deeper into how to implement the "mind" of a robot using Python and the rclpy library. This chapter focuses on bridging high-level AI logic with low-level ROS controllers, creating what we call the Agent-Controller bridge pattern.

Python is an excellent choice for implementing AI logic due to its rich ecosystem of machine learning and robotics libraries. The rclpy library provides Python bindings for ROS 2, allowing us to create sophisticated AI agents that can interact with the ROS 2 ecosystem seamlessly.

## 1. Understanding the Agent-Controller Bridge Pattern

The **Agent-Controller bridge pattern** is a design pattern that separates high-level decision-making logic (the agent) from low-level control execution (the controller). This separation allows for:

- Clear separation of concerns
- Independent development and testing
- Reusable control modules
- Scalable AI implementations

### Basic Architecture

```
[AI Agent] ←→ [Bridge Layer] ←→ [ROS Controllers]
   ↑              ↑               ↑
Decision      Communication     Execution
Making         (rclpy)         (ROS Nodes)
```

## 2. Creating an AI Agent Node

Let's create an AI agent that makes decisions based on sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

class RobotAIAgent(Node):
    def __init__(self):
        super().__init__('robot_ai_agent')

        # Subscribe to sensor data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        # Publisher for movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_callback)

        # Internal state
        self.safety_distance = 1.0  # meters
        self.obstacle_detected = False
        self.current_behavior = 'explore'  # explore, avoid, stop

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Find the minimum distance in the laser scan
        if len(msg.ranges) > 0:
            min_distance = min([r for r in msg.ranges if r > 0 and not r > msg.range_max], default=float('inf'))
            self.obstacle_detected = min_distance < self.safety_distance

            if self.obstacle_detected:
                self.current_behavior = 'avoid'
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m')
            else:
                self.current_behavior = 'explore'

    def decision_callback(self):
        """Make decisions based on current state"""
        cmd_msg = Twist()

        if self.current_behavior == 'explore':
            # Move forward at a moderate speed
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0
        elif self.current_behavior == 'avoid':
            # Implement obstacle avoidance behavior
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn to avoid obstacle
        else:
            # Stop by default
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        # Publish the command
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = RobotAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Advanced AI Integration

Let's create a more sophisticated AI agent that uses decision trees for behavior selection:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from enum import Enum
import math

class RobotBehavior(Enum):
    IDLE = "idle"
    EXPLORE = "explore"
    AVOID = "avoid"
    RETURN_HOME = "return_home"
    CHARGING = "charging"

class AdvancedAIAgent(Node):
    def __init__(self):
        super().__init__('advanced_ai_agent')

        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        self.battery_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_callback)

        # Internal state
        self.safety_distance = 0.8
        self.low_battery_threshold = 0.2  # 20%
        self.battery_level = 1.0
        self.obstacle_distances = []
        self.current_behavior = RobotBehavior.EXPLORE

    def laser_callback(self, msg):
        """Process laser scan data"""
        if len(msg.ranges) > 0:
            # Store obstacle distances for analysis
            self.obstacle_distances = [r for r in msg.ranges if 0 < r < msg.range_max]

            # Check for immediate obstacles
            if self.obstacle_distances:
                min_distance = min(self.obstacle_distances)
                if min_distance < self.safety_distance:
                    self.current_behavior = RobotBehavior.AVOID

    def battery_callback(self, msg):
        """Process battery data"""
        self.battery_level = msg.percentage
        if self.battery_level < self.low_battery_threshold:
            self.current_behavior = RobotBehavior.RETURN_HOME

    def decision_callback(self):
        """Advanced decision making using behavior tree logic"""
        cmd_msg = Twist()

        if self.current_behavior == RobotBehavior.RETURN_HOME:
            # Simple return home behavior
            cmd_msg.linear.x = -0.2  # Move backward toward home
            cmd_msg.angular.z = 0.0
            self.get_logger().info('Returning to charging station')
        elif self.current_behavior == RobotBehavior.AVOID:
            # Obstacle avoidance with random direction
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.6  # Turn to avoid obstacle
        elif self.current_behavior == RobotBehavior.EXPLORE:
            # Random walk exploration
            cmd_msg.linear.x = 0.4
            cmd_msg.angular.z = random.uniform(-0.3, 0.3)
        else:
            # Default: stop
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AdvancedAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Implementing Learning Capabilities

Let's add a simple learning mechanism to our AI agent:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from collections import deque

class LearningAIAgent(Node):
    def __init__(self):
        super().__init__('learning_ai_agent')

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.decision_callback)

        # Learning components
        self.safety_distance = 1.0
        self.action_history = deque(maxlen=100)  # Store recent actions
        self.reward_history = deque(maxlen=100)  # Store recent rewards
        self.q_table = {}  # Simple Q-learning table

        # State representation
        self.previous_state = None
        self.current_state = None

    def get_state(self, laser_data):
        """Convert laser data to discrete state representation"""
        if not laser_data:
            return "unknown"

        front_avg = np.mean(laser_data[:len(laser_data)//5])  # Front 20%
        left_avg = np.mean(laser_data[len(laser_data)//5:2*len(laser_data)//5])  # Next 20%
        right_avg = np.mean(laser_data[-len(laser_data)//5:])  # Last 20%

        # Discretize distances
        front_state = "close" if front_avg < self.safety_distance else "far"
        left_state = "close" if left_avg < self.safety_distance else "far"
        right_state = "close" if right_avg < self.safety_distance else "far"

        return f"{front_state}_{left_state}_{right_state}"

    def choose_action(self, state):
        """Choose action based on Q-learning policy"""
        if state not in self.q_table:
            # Initialize Q-values for new state
            self.q_table[state] = {"forward": 0.0, "left": 0.0, "right": 0.0, "stop": 0.0}

        # Simple epsilon-greedy policy (with high exploration for learning)
        if random.random() < 0.1:  # 10% exploration
            return random.choice(["forward", "left", "right", "stop"])
        else:
            # Choose action with highest Q-value
            return max(self.q_table[state], key=self.q_table[state].get)

    def update_q_value(self, state, action, reward, next_state, alpha=0.1, gamma=0.9):
        """Update Q-value using Q-learning formula"""
        if state not in self.q_table:
            self.q_table[state] = {"forward": 0.0, "left": 0.0, "right": 0.0, "stop": 0.0}
        if next_state not in self.q_table:
            self.q_table[next_state] = {"forward": 0.0, "left": 0.0, "right": 0.0, "stop": 0.0}

        current_q = self.q_table[state][action]
        max_next_q = max(self.q_table[next_state].values())

        # Q-learning update rule
        new_q = current_q + alpha * (reward + gamma * max_next_q - current_q)
        self.q_table[state][action] = new_q

    def calculate_reward(self, action, laser_data):
        """Calculate reward based on action and environment state"""
        if not laser_data:
            return 0.0

        min_distance = min(laser_data) if laser_data else float('inf')

        # Positive reward for moving forward when safe
        if action == "forward" and min_distance > self.safety_distance * 1.5:
            return 1.0

        # Negative reward for getting too close to obstacles
        if min_distance < self.safety_distance:
            return -10.0

        # Small negative reward for not moving forward
        if action != "forward":
            return -0.1

        return 0.1  # Small positive reward for safe forward movement

    def laser_callback(self, msg):
        """Process laser data and store for decision making"""
        self.current_state = self.get_state([r for r in msg.ranges if 0 < r < msg.range_max])

    def decision_callback(self):
        """Learning-based decision making"""
        if self.current_state is None:
            return

        # Choose action based on current state
        action = self.choose_action(self.current_state)

        # Execute action
        cmd_msg = Twist()
        if action == "forward":
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0
        elif action == "left":
            cmd_msg.linear.x = 0.1
            cmd_msg.angular.z = 0.3
        elif action == "right":
            cmd_msg.linear.x = 0.1
            cmd_msg.angular.z = -0.3
        else:  # stop
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd_msg)

        # Calculate reward for the action
        laser_data = [r for r in self.laser_subscriber._sub.msg_queue if r is not None]
        if laser_data:
            reward = self.calculate_reward(action, laser_data[-1].ranges)
        else:
            reward = 0.0

        # Update Q-values if we have a previous state
        if self.previous_state:
            self.update_q_value(self.previous_state, action, reward, self.current_state)

        # Store for next iteration
        self.previous_state = self.current_state
        self.action_history.append(action)
        self.reward_history.append(reward)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = LearningAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Best Practices for Python-ROS Integration

### Error Handling and Robustness

```python
def safe_execute_with_fallback(self, primary_action, fallback_action):
    """Execute primary action with fallback in case of failure"""
    try:
        return primary_action()
    except Exception as e:
        self.get_logger().error(f'Primary action failed: {e}, using fallback')
        return fallback_action()
```

### Resource Management

Always properly clean up resources when your node shuts down:

```python
def destroy_node(self):
    """Clean up resources before destroying node"""
    # Cancel timers
    self.timer.cancel()

    # Destroy publishers/subscribers
    self.cmd_vel_publisher.destroy()
    self.laser_subscriber.destroy()

    # Call parent destroy
    super().destroy_node()
```

## Summary

In this chapter, we've explored how to create sophisticated AI agents using Python and rclpy. We've covered:

- The Agent-Controller bridge pattern for separating AI logic from control execution
- Implementation of basic and advanced AI agents
- Introduction to learning mechanisms with Q-learning
- Best practices for robust Python-ROS integration

The bridge between high-level AI logic and low-level ROS controllers is crucial for creating intelligent robotic systems. In the next chapter, we'll explore how to define the physical structure of robots using URDF, which will allow our AI agents to interact with properly modeled robot bodies.

## Exercises

1. Implement a more sophisticated learning algorithm in the AI agent
2. Create a multi-agent system where multiple AI agents coordinate
3. Add computer vision capabilities to your AI agent using image topics