#!/usr/bin/env python3
"""
LLMTaskPlanner - ROS 2 Node for LLM-Based Task Planning

This node uses OpenAI's GPT-4o to interpret natural language commands and
decompose them into executable ROS 2 action sequences. It serves as the
cognitive planning layer that bridges high-level human instructions with
low-level robot actions.

Author: Humanoid Academy
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from manipulation_msgs.action import PickObject, PlaceObject
from rclpy.action import ActionClient
import openai
import json
import time
import os
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class TaskType(Enum):
    """Enumeration of supported task types."""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    COMPOSITE = "composite"
    QUERY = "query"


@dataclass
class TaskStep:
    """Represents a single step in a task plan."""
    action_type: str
    parameters: Dict[str, Any]
    description: str
    required_objects: List[str]
    safety_constraints: List[str]


@dataclass
class TaskPlan:
    """Represents a complete task plan."""
    task_id: str
    original_command: str
    task_type: TaskType
    steps: List[TaskStep]
    context: Dict[str, Any]
    safety_validation_passed: bool = False


class LLMTaskPlanner(Node):
    """
    ROS 2 Node that uses LLMs for natural language understanding and task planning.
    Converts high-level commands into executable ROS 2 action sequences.
    """

    def __init__(self):
        super().__init__('llm_task_planner')

        # Initialize parameters
        self.declare_parameter('gpt_model', 'gpt-4o')
        self.declare_parameter('max_context_tokens', 4096)
        self.declare_parameter('planning_timeout', 30.0)
        self.declare_parameter('api_key', os.getenv('OPENAI_API_KEY', ''))
        self.declare_parameter('enable_context_awareness', True)
        self.declare_parameter('safety_validation_enabled', True)

        # Get parameters
        self.gpt_model = self.get_parameter('gpt_model').value
        self.max_context_tokens = self.get_parameter('max_context_tokens').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.api_key = self.get_parameter('api_key').value
        self.enable_context_awareness = self.get_parameter('enable_context_awareness').value
        self.safety_validation_enabled = self.get_parameter('safety_validation_enabled').value

        # Validate API key
        if not self.api_key:
            self.get_logger().error("OpenAI API key not found. Please set OPENAI_API_KEY environment variable.")
            raise ValueError("OpenAI API key is required")

        # Set OpenAI API key
        openai.api_key = self.api_key

        # Initialize action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pick_client = ActionClient(self, PickObject, 'pick_object')
        self.place_client = ActionClient(self, PlaceObject, 'place_object')

        # ROS 2 Publishers
        self.task_plan_pub = self.create_publisher(
            String,
            'task_plan',
            QoSProfile(depth=10)
        )

        self.status_pub = self.create_publisher(
            String,
            'planning_status',
            QoSProfile(depth=10)
        )

        # ROS 2 Subscribers
        self.command_sub = self.create_subscription(
            String,
            'voice_interpretation',
            self.command_callback,
            QoSProfile(depth=10)
        )

        # Initialize context management
        self.current_environment_context = {
            'objects': {},
            'locations': {},
            'robot_state': {},
            'time': time.time()
        }

        # Initialize safety validator
        self.safety_validator = SafetyValidator(self)

        # Statistics
        self.stats = {
            'total_plans_generated': 0,
            'total_errors': 0,
            'avg_planning_time': 0.0,
            'success_rate': 0.0
        }

        # Create timer for periodic statistics reporting
        self.stats_timer = self.create_timer(60.0, self.report_statistics)

        self.get_logger().info(f"LLMTaskPlanner initialized with model: {self.gpt_model}")

    def command_callback(self, msg: String):
        """
        Callback function for receiving interpreted voice commands.
        Processes the command and generates a task plan.
        """
        try:
            command = msg.data
            self.get_logger().info(f"Received command for planning: {command}")

            # Publish status update
            status_msg = String()
            status_msg.data = f"Processing command: {command}"
            self.status_pub.publish(status_msg)

            # Generate task plan using LLM
            start_time = time.time()
            task_plan = self.generate_task_plan(command)

            if task_plan:
                # Validate safety if enabled
                if self.safety_validation_enabled:
                    task_plan.safety_validation_passed = self.safety_validator.validate_task_plan(task_plan)
                    if not task_plan.safety_validation_passed:
                        self.get_logger().warn("Task plan failed safety validation")
                        return

                # Publish the task plan
                plan_msg = String()
                plan_msg.data = json.dumps({
                    'task_id': task_plan.task_id,
                    'original_command': task_plan.original_command,
                    'task_type': task_plan.task_type.value,
                    'steps': [
                        {
                            'action_type': step.action_type,
                            'parameters': step.parameters,
                            'description': step.description,
                            'required_objects': step.required_objects,
                            'safety_constraints': step.safety_constraints
                        } for step in task_plan.steps
                    ],
                    'context': task_plan.context,
                    'safety_validation_passed': task_plan.safety_validation_passed
                })
                self.task_plan_pub.publish(plan_msg)

                # Update statistics
                planning_time = time.time() - start_time
                self.stats['total_plans_generated'] += 1
                self.stats['avg_planning_time'] = (
                    (self.stats['avg_planning_time'] * (self.stats['total_plans_generated'] - 1) + planning_time)
                    / self.stats['total_plans_generated']
                )
                self.stats['success_rate'] = (
                    self.stats['total_plans_generated'] /
                    (self.stats['total_plans_generated'] + self.stats['total_errors'])
                ) if (self.stats['total_plans_generated'] + self.stats['total_errors']) > 0 else 0.0

                self.get_logger().info(f"Published task plan with {len(task_plan.steps)} steps")

        except Exception as e:
            self.get_logger().error(f"Error in command callback: {str(e)}")
            self.stats['total_errors'] += 1

            # Publish error status
            error_msg = String()
            error_msg.data = f"Error processing command: {str(e)}"
            self.status_pub.publish(error_msg)

    def generate_task_plan(self, command: str) -> Optional[TaskPlan]:
        """
        Generate a task plan from a natural language command using GPT-4o.

        Args:
            command: Natural language command to plan

        Returns:
            TaskPlan object or None if planning failed
        """
        try:
            # Prepare context for the LLM
            context_prompt = self._build_context_prompt(command)

            # Call GPT-4o to generate the task plan
            response = openai.ChatCompletion.create(
                model=self.gpt_model,
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are an expert task planner for a humanoid robot. Your job is to decompose "
                            "natural language commands into executable action sequences. Each action should "
                            "be specific, actionable, and include necessary parameters. "
                            "Respond in JSON format with the following structure: "
                            "{"
                            "  'task_type': 'navigation|manipulation|composite|query',"
                            "  'steps': ["
                            "    {"
                            "      'action_type': 'navigate_to|pick_object|place_object|move_arm|etc',"
                            "      'parameters': {key: value},"
                            "      'description': 'Human-readable description',"
                            "      'required_objects': ['object1', 'object2'],"
                            "      'safety_constraints': ['constraint1', 'constraint2']"
                            "    }"
                            "  ],"
                            "  'context': {key: value}"
                            "}"
                        )
                    },
                    {
                        "role": "user",
                        "content": context_prompt
                    }
                ],
                temperature=0.3,
                max_tokens=1000,
                timeout=self.planning_timeout
            )

            # Parse the response
            response_text = response.choices[0].message.content.strip()

            # Clean up response if it contains markdown code block markers
            if response_text.startswith('```json'):
                response_text = response_text[7:]  # Remove ```json
            if response_text.endswith('```'):
                response_text = response_text[:-3]  # Remove ```

            # Parse JSON response
            plan_data = json.loads(response_text)

            # Create TaskPlan object
            task_plan = TaskPlan(
                task_id=f"plan_{int(time.time())}",
                original_command=command,
                task_type=TaskType(plan_data['task_type']),
                steps=[],
                context=plan_data.get('context', {})
            )

            # Create TaskStep objects from the plan data
            for step_data in plan_data['steps']:
                step = TaskStep(
                    action_type=step_data['action_type'],
                    parameters=step_data['parameters'],
                    description=step_data['description'],
                    required_objects=step_data.get('required_objects', []),
                    safety_constraints=step_data.get('safety_constraints', [])
                )
                task_plan.steps.append(step)

            return task_plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing LLM response as JSON: {str(e)}")
            self.stats['total_errors'] += 1
            return None
        except Exception as e:
            self.get_logger().error(f"Error generating task plan: {str(e)}")
            self.stats['total_errors'] += 1
            return None

    def _build_context_prompt(self, command: str) -> str:
        """
        Build a context prompt for the LLM with environmental and robot state information.

        Args:
            command: The natural language command

        Returns:
            Formatted context prompt
        """
        context_info = {
            "command": command,
            "environment_objects": self.current_environment_context['objects'],
            "available_locations": self.current_environment_context['locations'],
            "robot_capabilities": {
                "navigation": True,
                "manipulation": True,
                "grasping": True,
                "speech": True
            },
            "robot_state": self.current_environment_context['robot_state'],
            "current_time": self.current_environment_context['time']
        }

        prompt = (
            f"Command: {command}\n\n"
            f"Environmental Context:\n"
            f"- Available objects: {json.dumps(context_info['environment_objects'], indent=2)}\n"
            f"- Available locations: {json.dumps(context_info['available_locations'], indent=2)}\n"
            f"- Robot capabilities: {json.dumps(context_info['robot_capabilities'], indent=2)}\n"
            f"- Current robot state: {json.dumps(context_info['robot_state'], indent=2)}\n\n"
            f"Please decompose this command into a sequence of executable actions for a humanoid robot. "
            f"Each action should be specific and include necessary parameters for ROS 2 action execution. "
            f"Consider safety constraints and environmental context when generating the plan."
        )

        return prompt

    def update_environment_context(self, context_update: Dict[str, Any]):
        """
        Update the environmental context with new information.

        Args:
            context_update: Dictionary containing context updates
        """
        for key, value in context_update.items():
            self.current_environment_context[key] = value
        self.current_environment_context['time'] = time.time()

    def execute_task_plan(self, task_plan: TaskPlan) -> bool:
        """
        Execute a generated task plan by calling appropriate ROS 2 action servers.

        Args:
            task_plan: The task plan to execute

        Returns:
            True if execution was successful, False otherwise
        """
        try:
            self.get_logger().info(f"Executing task plan with {len(task_plan.steps)} steps")

            for i, step in enumerate(task_plan.steps):
                self.get_logger().info(f"Executing step {i+1}/{len(task_plan.steps)}: {step.description}")

                # Execute the action based on type
                success = self._execute_action_step(step)

                if not success:
                    self.get_logger().error(f"Failed to execute step: {step.description}")
                    return False

                # Small delay between steps to allow for system stabilization
                time.sleep(0.5)

            self.get_logger().info("Task plan executed successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"Error executing task plan: {str(e)}")
            return False

    def _execute_action_step(self, step: TaskStep) -> bool:
        """
        Execute a single action step.

        Args:
            step: The action step to execute

        Returns:
            True if execution was successful, False otherwise
        """
        try:
            if step.action_type == 'navigate_to':
                return self._execute_navigation(step.parameters)
            elif step.action_type == 'pick_object':
                return self._execute_manipulation_pick(step.parameters)
            elif step.action_type == 'place_object':
                return self._execute_manipulation_place(step.parameters)
            elif step.action_type == 'move_arm':
                return self._execute_arm_movement(step.parameters)
            else:
                self.get_logger().warn(f"Unknown action type: {step.action_type}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error executing action step: {str(e)}")
            return False

    def _execute_navigation(self, params: Dict[str, Any]) -> bool:
        """
        Execute navigation action.

        Args:
            params: Navigation parameters

        Returns:
            True if successful, False otherwise
        """
        try:
            # Wait for action server
            self.nav_client.wait_for_server()

            # Create goal
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = params.get('frame_id', 'map')
            goal.pose.pose.position.x = params.get('x', 0.0)
            goal.pose.pose.position.y = params.get('y', 0.0)
            goal.pose.pose.position.z = params.get('z', 0.0)
            goal.pose.pose.orientation.x = params.get('qx', 0.0)
            goal.pose.pose.orientation.y = params.get('qy', 0.0)
            goal.pose.pose.orientation.z = params.get('qz', 0.0)
            goal.pose.pose.orientation.w = params.get('qw', 1.0)

            # Send goal
            future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Navigation goal rejected")
                return False

            # Get result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result.result:
                self.get_logger().info("Navigation completed successfully")
                return True
            else:
                self.get_logger().error("Navigation failed")
                return False

        except Exception as e:
            self.get_logger().error(f"Navigation execution error: {str(e)}")
            return False

    def _execute_manipulation_pick(self, params: Dict[str, Any]) -> bool:
        """
        Execute pick object action.

        Args:
            params: Pick parameters

        Returns:
            True if successful, False otherwise
        """
        try:
            # Wait for action server
            self.pick_client.wait_for_server()

            # Create goal
            goal = PickObject.Goal()
            goal.object_name = params.get('object_name', '')
            goal.object_pose.header.frame_id = params.get('frame_id', 'base_link')
            goal.object_pose.pose.position.x = params.get('x', 0.0)
            goal.object_pose.pose.position.y = params.get('y', 0.0)
            goal.object_pose.pose.position.z = params.get('z', 0.0)

            # Send goal
            future = self.pick_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Pick goal rejected")
                return False

            # Get result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result.result and result.result.success:
                self.get_logger().info("Pick operation completed successfully")
                return True
            else:
                self.get_logger().error("Pick operation failed")
                return False

        except Exception as e:
            self.get_logger().error(f"Pick execution error: {str(e)}")
            return False

    def _execute_manipulation_place(self, params: Dict[str, Any]) -> bool:
        """
        Execute place object action.

        Args:
            params: Place parameters

        Returns:
            True if successful, False otherwise
        """
        try:
            # Wait for action server
            self.place_client.wait_for_server()

            # Create goal
            goal = PlaceObject.Goal()
            goal.object_name = params.get('object_name', '')
            goal.place_pose.header.frame_id = params.get('frame_id', 'base_link')
            goal.place_pose.pose.position.x = params.get('x', 0.0)
            goal.place_pose.pose.position.y = params.get('y', 0.0)
            goal.place_pose.pose.position.z = params.get('z', 0.0)

            # Send goal
            future = self.place_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Place goal rejected")
                return False

            # Get result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result.result and result.result.success:
                self.get_logger().info("Place operation completed successfully")
                return True
            else:
                self.get_logger().error("Place operation failed")
                return False

        except Exception as e:
            self.get_logger().error(f"Place execution error: {str(e)}")
            return False

    def _execute_arm_movement(self, params: Dict[str, Any]) -> bool:
        """
        Execute arm movement action (placeholder implementation).

        Args:
            params: Arm movement parameters

        Returns:
            True if successful, False otherwise
        """
        try:
            # This would interface with actual arm control
            self.get_logger().info(f"Executing arm movement: {params}")
            # In a real implementation, this would call arm control services
            return True
        except Exception as e:
            self.get_logger().error(f"Arm movement execution error: {str(e)}")
            return False

    def report_statistics(self):
        """Report periodic statistics about task planning performance."""
        self.get_logger().info(
            f"LLM Task Planner Statistics - "
            f"Plans Generated: {self.stats['total_plans_generated']}, "
            f"Errors: {self.stats['total_errors']}, "
            f"Avg Planning Time: {self.stats['avg_planning_time']:.2f}s, "
            f"Success Rate: {self.stats['success_rate']:.2f}"
        )

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        super().destroy_node()


class SafetyValidator:
    """
    Validates task plans for safety before execution.
    """

    def __init__(self, node: LLMTaskPlanner):
        self.node = node

    def validate_task_plan(self, task_plan: TaskPlan) -> bool:
        """
        Validate a task plan for safety compliance.

        Args:
            task_plan: The task plan to validate

        Returns:
            True if plan is safe, False otherwise
        """
        try:
            # Check for navigation safety
            for step in task_plan.steps:
                if step.action_type == 'navigate_to':
                    if not self._validate_navigation_safety(step.parameters):
                        return False
                elif step.action_type == 'pick_object':
                    if not self._validate_manipulation_safety(step.parameters):
                        return False
                elif step.action_type == 'place_object':
                    if not self._validate_manipulation_safety(step.parameters):
                        return False

            # Check overall plan safety
            if not self._validate_plan_coherence(task_plan):
                return False

            return True

        except Exception as e:
            self.node.get_logger().error(f"Error in safety validation: {str(e)}")
            return False

    def _validate_navigation_safety(self, params: Dict[str, Any]) -> bool:
        """
        Validate navigation parameters for safety.

        Args:
            params: Navigation parameters

        Returns:
            True if safe, False otherwise
        """
        # Check for valid coordinates
        x, y = params.get('x', 0), params.get('y', 0)
        if abs(x) > 100 or abs(y) > 100:  # Reasonable limits for indoor navigation
            self.node.get_logger().error(f"Navigation target too far: ({x}, {y})")
            return False

        # In a real implementation, check against known obstacles and safe zones
        return True

    def _validate_manipulation_safety(self, params: Dict[str, Any]) -> bool:
        """
        Validate manipulation parameters for safety.

        Args:
            params: Manipulation parameters

        Returns:
            True if safe, False otherwise
        """
        # Check for valid object names and positions
        obj_name = params.get('object_name', '')
        if not obj_name:
            self.node.get_logger().error("Manipulation target has no object name")
            return False

        # Check position limits
        x, y, z = params.get('x', 0), params.get('y', 0), params.get('z', 0)
        if z < 0:  # Can't manipulate objects below ground level
            self.node.get_logger().error(f"Invalid manipulation height: {z}")
            return False

        return True

    def _validate_plan_coherence(self, task_plan: TaskPlan) -> bool:
        """
        Validate that the task plan is coherent and executable.

        Args:
            task_plan: The task plan to validate

        Returns:
            True if coherent, False otherwise
        """
        # Check that required objects exist in environment context
        for step in task_plan.steps:
            for obj in step.required_objects:
                if obj not in self.node.current_environment_context['objects']:
                    self.node.get_logger().warn(f"Required object not in environment: {obj}")
                    # This might be acceptable depending on task context

        return True


def main(args=None):
    """Main function to run the LLMTaskPlanner node."""
    rclpy.init(args=args)

    try:
        node = LLMTaskPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running LLMTaskPlanner: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()