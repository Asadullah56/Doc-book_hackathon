# Code Examples for Module 1: The Robotic Nervous System (ROS 2)

This directory contains the code examples referenced in Module 1 of the Physical AI & Humanoid Robotics textbook.

## Available Examples

### 1. Publisher-Subscriber Telemetry (`publisher_subscriber_telemetry.py`)
- Demonstrates basic ROS 2 communication concepts
- Shows how to create publisher and subscriber nodes
- Simulates robot telemetry data exchange

### 2. Agent-Controller Bridge (`agent_controller_bridge.py`)
- Implements the Agent-Controller bridge pattern
- Shows separation of high-level AI logic from low-level control
- Demonstrates state management and decision making

### 3. Humanoid Robot URDF (`simple_humanoid.urdf`)
- Simplified humanoid robot model for educational purposes
- Includes torso, head, arms, and legs with appropriate joints
- Demonstrates URDF structure and best practices

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- rclpy library (comes with ROS 2 installation)

## Running the Examples

### Publisher-Subscriber Example:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to the examples directory
cd doc-book/docs/module-1/code-examples/

# Run the example
python3 publisher_subscriber_telemetry.py
```

### Agent-Controller Bridge Example:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to the examples directory
cd doc-book/docs/module-1/code-examples/

# Run the example
python3 agent_controller_bridge.py
```

### Validating URDF:
```bash
# Using check_urdf tool
check_urdf simple_humanoid.urdf

# Using xacro (if available)
ros2 run xacro xacro simple_humanoid.urdf
```

## Understanding the Agent-Controller Bridge Pattern

The `agent_controller_bridge.py` example demonstrates the core concept from Chapter 2:

1. **Agent Layer**: High-level decision making (the "mind")
2. **Controller Layer**: Low-level command execution (the "body")
3. **Communication**: Standard ROS 2 messages for coordination

This pattern allows for:
- Clear separation of concerns
- Independent development and testing
- Reusable control modules
- Scalable AI implementations