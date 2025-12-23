---
sidebar_position: 2
---

# Chapter 2: Cognitive Planning: LLMs as Task Planners

<button>Personalize Content</button>
<button>Translate to Urdu</button>

<div class="humanoid-academy-banner">
  <h3>Humanoid Academy - Advanced Robotics Education</h3>
  <p>Empowering the next generation of AI roboticists with cutting-edge technology and practical implementation</p>
</div>

## Introduction to LLM-Based Cognitive Planning

Large Language Models (LLMs) have emerged as powerful cognitive planning systems for robotics, enabling natural language understanding and task decomposition in complex robotic environments (Achiam et al., 2023). The integration of OpenAI's GPT-4o with ROS 2 creates a sophisticated middleware layer that translates high-level human commands into executable robotic actions.

Cognitive planning in robotics involves breaking down complex tasks into manageable subtasks, considering environmental constraints, robot capabilities, and safety requirements (Brohan et al., 2022). LLMs excel at this by leveraging their vast knowledge base and reasoning capabilities to generate appropriate action sequences for diverse scenarios.

The challenge lies in bridging the gap between natural language commands and precise robotic actions. This chapter explores the implementation of LLM-based task planning systems that can interpret commands like "Clean the table" and decompose them into specific ROS 2 action goals including navigation, object detection, manipulation, and cleanup sequences.

## Natural Language Processing for Task Decomposition

The foundation of cognitive planning lies in effective natural language processing that can understand user intent and translate it into structured robotic tasks. This process involves multiple stages of linguistic analysis and semantic understanding.

### Language Understanding Pipeline

The language understanding pipeline processes natural language commands through several stages:

```python
import openai
import json
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from enum import Enum

class TaskType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    INTERACTION = "interaction"
    SYSTEM = "system"

@dataclass
class TaskStep:
    """Represents a single step in a decomposed task"""
    id: str
    description: str
    task_type: TaskType
    ros_action: str
    parameters: Dict[str, Any]
    dependencies: List[str]  # IDs of steps that must be completed first
    success_criteria: str
    timeout: float = 30.0

@dataclass
class TaskPlan:
    """Represents a complete decomposed task plan"""
    original_command: str
    intent: str
    steps: List[TaskStep]
    priority: int = 0
    estimated_duration: float = 0.0

class NLPProcessor:
    """Processes natural language commands for robotic task planning"""

    def __init__(self, openai_api_key: str):
        openai.api_key = openai_api_key
        self.task_type_mapping = {
            'move': TaskType.NAVIGATION,
            'go': TaskType.NAVIGATION,
            'navigate': TaskType.NAVIGATION,
            'pick': TaskType.MANIPULATION,
            'grasp': TaskType.MANIPULATION,
            'take': TaskType.MANIPULATION,
            'place': TaskType.MANIPULATION,
            'put': TaskType.MANIPULATION,
            'see': TaskType.PERCEPTION,
            'look': TaskType.PERCEPTION,
            'find': TaskType.PERCEPTION,
            'detect': TaskType.PERCEPTION,
            'talk': TaskType.INTERACTION,
            'greet': TaskType.INTERACTION,
            'interact': TaskType.INTERACTION,
        }

    def process_command(self, command: str) -> TaskPlan:
        """Process a natural language command into a task plan"""
        # Step 1: Intent recognition
        intent = self.recognize_intent(command)

        # Step 2: Entity extraction
        entities = self.extract_entities(command)

        # Step 3: Task decomposition using LLM
        task_plan = self.decompose_task(command, intent, entities)

        return task_plan

    def recognize_intent(self, command: str) -> str:
        """Recognize the high-level intent from the command"""
        # Use GPT-4o to understand the intent
        prompt = f"""
        Analyze the following robotic command and identify the high-level intent:
        Command: "{command}"

        Provide only the intent as a short phrase (e.g., "clean table", "fetch object", "navigate to location").
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=50,
                temperature=0.1
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            # Fallback to simple keyword matching
            return self.fallback_intent_recognition(command)

    def extract_entities(self, command: str) -> Dict[str, List[str]]:
        """Extract entities (objects, locations, people) from the command"""
        prompt = f"""
        Extract entities from the following robotic command:
        Command: "{command}"

        Return a JSON object with keys: objects, locations, people, quantities.
        Example: {{"objects": ["cup", "table"], "locations": ["kitchen"], "people": ["John"], "quantities": ["2"]}}
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=100,
                temperature=0.1
            )

            # Parse the JSON response
            entities_text = response.choices[0].message.content.strip()
            if entities_text.startswith('```json'):
                entities_text = entities_text[7:entities_text.rfind('```')].strip()

            return json.loads(entities_text)
        except Exception as e:
            # Fallback to simple extraction
            return self.fallback_entity_extraction(command)

    def decompose_task(self, command: str, intent: str, entities: Dict[str, List[str]]) -> TaskPlan:
        """Decompose a high-level command into executable steps using LLM"""
        prompt = f"""
        Decompose the following robotic command into a sequence of executable steps:
        Command: "{command}"
        Intent: "{intent}"
        Entities: {json.dumps(entities)}

        Return a JSON array of steps, each with: id, description, task_type, ros_action, parameters, dependencies, success_criteria.

        Example format:
        [
            {{
                "id": "step_1",
                "description": "Navigate to the kitchen",
                "task_type": "navigation",
                "ros_action": "nav2_msgs/NavigateToPose",
                "parameters": {{"target_pose": [1.0, 2.0, 0.0]}},
                "dependencies": [],
                "success_criteria": "Robot reaches the kitchen location"
            }}
        ]

        Ensure the steps are appropriate for a humanoid robot and consider safety constraints.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=500,
                temperature=0.3
            )

            steps_text = response.choices[0].message.content.strip()
            if steps_text.startswith('```json'):
                steps_text = steps_text[7:steps_text.rfind('```')].strip()

            steps_data = json.loads(steps_text)

            # Convert to TaskStep objects
            steps = []
            for step_data in steps_data:
                step = TaskStep(
                    id=step_data['id'],
                    description=step_data['description'],
                    task_type=TaskType(step_data['task_type']),
                    ros_action=step_data['ros_action'],
                    parameters=step_data['parameters'],
                    dependencies=step_data['dependencies'],
                    success_criteria=step_data['success_criteria']
                )
                steps.append(step)

            return TaskPlan(
                original_command=command,
                intent=intent,
                steps=steps
            )
        except Exception as e:
            # Fallback to simple decomposition
            return self.fallback_task_decomposition(command, intent, entities)

    def fallback_intent_recognition(self, command: str) -> str:
        """Fallback intent recognition using keyword matching"""
        command_lower = command.lower()

        if any(word in command_lower for word in ['clean', 'tidy', 'organize']):
            return "clean"
        elif any(word in command_lower for word in ['fetch', 'bring', 'get', 'pick']):
            return "fetch"
        elif any(word in command_lower for word in ['go', 'move', 'navigate', 'walk']):
            return "navigate"
        else:
            return "general"

    def fallback_entity_extraction(self, command: str) -> Dict[str, List[str]]:
        """Fallback entity extraction using keyword matching"""
        entities = {
            'objects': [],
            'locations': [],
            'people': [],
            'quantities': []
        }

        command_lower = command.lower()

        # Simple object detection
        objects = ['cup', 'bottle', 'book', 'box', 'table', 'chair', 'plate', 'glass', 'fork', 'spoon']
        for obj in objects:
            if obj in command_lower:
                entities['objects'].append(obj)

        # Simple location detection
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'hallway', 'dining room']
        for loc in locations:
            if loc in command_lower:
                entities['locations'].append(loc)

        return entities

    def fallback_task_decomposition(self, command: str, intent: str, entities: Dict[str, List[str]]) -> TaskPlan:
        """Fallback task decomposition for when LLM fails"""
        # Simple decomposition based on intent
        steps = []

        if intent == "clean":
            steps = [
                TaskStep(
                    id="clean_1",
                    description="Navigate to the area to be cleaned",
                    task_type=TaskType.NAVIGATION,
                    ros_action="nav2_msgs/NavigateToPose",
                    parameters={"target_pose": [0.0, 0.0, 0.0]},
                    dependencies=[],
                    success_criteria="Robot reaches the cleaning area"
                ),
                TaskStep(
                    id="clean_2",
                    description="Detect objects to be cleaned",
                    task_type=TaskType.PERCEPTION,
                    ros_action="sensor_msgs/Image",
                    parameters={"sensor": "camera"},
                    dependencies=["clean_1"],
                    success_criteria="Objects identified in the scene"
                )
            ]
        elif intent == "fetch":
            steps = [
                TaskStep(
                    id="fetch_1",
                    description="Identify the object to fetch",
                    task_type=TaskType.PERCEPTION,
                    ros_action="sensor_msgs/Image",
                    parameters={"sensor": "camera"},
                    dependencies=[],
                    success_criteria="Target object located"
                )
            ]

        return TaskPlan(
            original_command=command,
            intent=intent,
            steps=steps
        )
```

### Context-Aware Task Planning

Modern cognitive planning systems incorporate context awareness to improve task decomposition accuracy and relevance:

```python
class ContextAwarePlanner:
    """Enhances task planning with contextual information"""

    def __init__(self, nlp_processor: NLPProcessor):
        self.nlp_processor = nlp_processor
        self.context_history = []
        self.robot_capabilities = {}
        self.environment_state = {}
        self.user_preferences = {}

    def plan_with_context(self, command: str, context: Dict[str, Any]) -> TaskPlan:
        """Plan tasks considering contextual information"""
        # Update context with new information
        self.update_context(context)

        # Enhance command with context
        enhanced_command = self.enhance_command_with_context(command)

        # Process the enhanced command
        task_plan = self.nlp_processor.process_command(enhanced_command)

        # Apply context-specific modifications
        self.apply_context_modifications(task_plan)

        return task_plan

    def update_context(self, context: Dict[str, Any]):
        """Update the planner's context with new information"""
        self.context_history.append(context)

        # Keep only recent context (last 10 interactions)
        if len(self.context_history) > 10:
            self.context_history = self.context_history[-10:]

    def enhance_command_with_context(self, command: str) -> str:
        """Enhance command with contextual information"""
        # Add context to command for better understanding
        context_info = []

        # Add robot state information
        if self.robot_capabilities:
            context_info.append(f"Robot capabilities: {self.robot_capabilities}")

        # Add environment state
        if self.environment_state:
            context_info.append(f"Environment: {self.environment_state}")

        # Add recent history
        if self.context_history:
            recent_context = self.context_history[-1]
            if 'location' in recent_context:
                context_info.append(f"Current location: {recent_context['location']}")

        if context_info:
            enhanced_command = f"Context: {'; '.join(context_info)}. Command: {command}"
            return enhanced_command

        return command

    def apply_context_modifications(self, task_plan: TaskPlan):
        """Apply context-specific modifications to the task plan"""
        # Modify task plan based on context
        for step in task_plan.steps:
            # Apply safety modifications based on environment
            if self.environment_state.get('obstacles'):
                step.parameters['avoid_obstacles'] = True

            # Apply capability modifications based on robot state
            if not self.robot_capabilities.get('manipulation'):
                if step.task_type == TaskType.MANIPULATION:
                    step.task_type = TaskType.NAVIGATION
                    step.description = f"Navigate to {step.description}"
```

## GPT-4o Integration for Task Decomposition

GPT-4o provides exceptional capabilities for natural language understanding and task decomposition, making it ideal for cognitive planning applications. The integration involves careful prompt engineering and response processing to ensure reliable task generation.

### Prompt Engineering Strategies

Effective prompt engineering is crucial for reliable task decomposition:

```python
class PromptEngineer:
    """Handles prompt engineering for GPT-4o task decomposition"""

    def __init__(self):
        self.system_prompt = """
        You are an expert robotic task planner. Your role is to decompose high-level human commands
        into detailed, executable robotic action sequences. Consider:
        - Safety constraints for humanoid robots
        - Physical limitations and capabilities
        - Environmental factors and obstacles
        - Sequential dependencies between actions
        - Success criteria for each action
        - Appropriate ROS 2 action types
        """

    def create_decomposition_prompt(self, command: str, intent: str, entities: Dict[str, List[str]],
                                   robot_capabilities: Dict[str, Any], environment_state: Dict[str, Any]) -> str:
        """Create a comprehensive prompt for task decomposition"""

        capabilities_str = json.dumps(robot_capabilities, indent=2)
        environment_str = json.dumps(environment_state, indent=2)

        prompt = f"""
        {self.system_prompt}

        ROBOT CAPABILITIES:
        {capabilities_str}

        ENVIRONMENT STATE:
        {environment_str}

        TASK DECOMPOSITION REQUEST:
        Command: "{command}"
        Intent: "{intent}"
        Entities: {json.dumps(entities)}

        Decompose this command into a sequence of executable steps following this JSON format:

        [
            {{
                "id": "unique_step_id",
                "description": "Clear description of what the robot should do",
                "task_type": "navigation|manipulation|perception|interaction|system",
                "ros_action": "ROS 2 action type (e.g., nav2_msgs/NavigateToPose)",
                "parameters": {{"param1": "value1", "param2": "value2"}},
                "dependencies": ["id_of_step_that_must_complete_first"],
                "success_criteria": "How to verify this step completed successfully",
                "timeout": 30.0
            }}
        ]

        REQUIREMENTS:
        1. Each step must be executable by the robot given its capabilities
        2. Steps must follow logical sequence with proper dependencies
        3. Include safety checks and obstacle avoidance where appropriate
        4. Consider the environment state in planning
        5. Provide specific parameter values where possible
        6. Include realistic timeouts for each step
        """

        return prompt

    def create_validation_prompt(self, task_plan_json: str, command: str) -> str:
        """Create prompt to validate the generated task plan"""

        prompt = f"""
        Validate the following robotic task plan for the command: "{command}"

        Task Plan:
        {task_plan_json}

        Check for:
        1. Logical sequence of steps
        2. Proper dependencies between steps
        3. Feasibility given robot capabilities
        4. Safety considerations
        5. Completeness in addressing the original command

        If valid, return the original JSON. If invalid, return corrected JSON with explanations.
        """

        return prompt
```

### Response Processing and Validation

Processing GPT-4o responses requires robust validation to ensure reliability:

```python
import re
import json
from typing import Union

class ResponseProcessor:
    """Processes and validates GPT-4o responses for task decomposition"""

    def __init__(self):
        self.max_retries = 3

    def process_response(self, response_content: str) -> List[Dict[str, Any]]:
        """Process and validate GPT-4o response"""

        # Clean the response content
        cleaned_content = self.clean_response(response_content)

        # Extract JSON from the response
        json_match = self.extract_json(cleaned_content)

        if not json_match:
            raise ValueError("No valid JSON found in response")

        # Parse the JSON
        try:
            task_steps = json.loads(json_match)
        except json.JSONDecodeError:
            raise ValueError("Invalid JSON format in response")

        # Validate the structure
        self.validate_task_structure(task_steps)

        return task_steps

    def clean_response(self, content: str) -> str:
        """Clean response content by removing markdown formatting"""
        # Remove markdown code block markers
        content = re.sub(r'```json\s*', '', content)
        content = re.sub(r'```\s*$', '', content)
        content = content.strip()

        return content

    def extract_json(self, content: str) -> Optional[str]:
        """Extract JSON from content that may contain other text"""
        # Look for JSON array pattern
        json_match = re.search(r'\[\s*\{.*\}\s*\]', content, re.DOTALL)

        if json_match:
            return json_match.group(0)

        # Look for JSON object pattern
        json_match = re.search(r'\{.*\}', content, re.DOTALL)

        if json_match:
            return json_match.group(0)

        return None

    def validate_task_structure(self, task_steps: List[Dict[str, Any]]):
        """Validate the structure of task steps"""
        required_fields = ['id', 'description', 'task_type', 'ros_action', 'parameters', 'dependencies', 'success_criteria']

        for i, step in enumerate(task_steps):
            if not isinstance(step, dict):
                raise ValueError(f"Step {i} is not a dictionary")

            for field in required_fields:
                if field not in step:
                    raise ValueError(f"Step {i} missing required field: {field}")

            # Validate task type
            if step['task_type'] not in ['navigation', 'manipulation', 'perception', 'interaction', 'system']:
                raise ValueError(f"Step {i} has invalid task_type: {step['task_type']}")

            # Validate parameters is a dictionary
            if not isinstance(step['parameters'], dict):
                raise ValueError(f"Step {i} parameters must be a dictionary")

            # Validate dependencies is a list
            if not isinstance(step['dependencies'], list):
                raise ValueError(f"Step {i} dependencies must be a list")

            # Validate timeout is a number
            timeout = step.get('timeout', 30.0)
            if not isinstance(timeout, (int, float)) or timeout <= 0:
                raise ValueError(f"Step {i} timeout must be a positive number")

    def validate_with_retry(self, command: str, task_plan_json: str,
                           validation_func: callable) -> Union[List[Dict[str, Any]], None]:
        """Validate task plan with retry logic"""

        for attempt in range(self.max_retries):
            try:
                task_steps = self.process_response(task_plan_json)

                # Apply custom validation
                if validation_func(task_steps, command):
                    return task_steps
                else:
                    raise ValueError("Custom validation failed")

            except Exception as e:
                if attempt == self.max_retries - 1:
                    # Final attempt failed
                    return None
                else:
                    # Retry with error feedback
                    continue

        return None
```

## LLM Middleware Architecture

The LLM middleware serves as the bridge between natural language commands and ROS 2 action execution, providing a scalable and maintainable architecture for cognitive planning.

### Middleware Components

```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time

class LLMTaskPlanner(Node):
    """Main LLM-based task planning node for ROS 2"""

    def __init__(self):
        super().__init__('llm_task_planner')

        # Declare parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('max_concurrent_tasks', 1)
        self.declare_parameter('task_queue_size', 10)
        self.declare_parameter('response_timeout', 30.0)

        # Initialize components
        api_key = self.get_parameter('openai_api_key').value
        if not api_key:
            self.get_logger().error("OpenAI API key not provided")
            return

        self.nlp_processor = NLPProcessor(api_key)
        self.context_planner = ContextAwarePlanner(self.nlp_processor)
        self.prompt_engineer = PromptEngineer()
        self.response_processor = ResponseProcessor()

        # Task management
        self.max_concurrent_tasks = self.get_parameter('max_concurrent_tasks').value
        self.response_timeout = self.get_parameter('response_timeout').value
        self.active_tasks = {}
        self.task_queue = asyncio.Queue(maxsize=self.get_parameter('task_queue_size').value)

        # ROS 2 interfaces
        self.command_sub = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        self.task_status_pub = self.create_publisher(
            String,
            'task_status',
            10
        )

        # Thread pool for LLM processing
        self.executor = ThreadPoolExecutor(max_workers=2)

        # Start task processing
        self.processing_thread = threading.Thread(target=self.process_task_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info("LLM Task Planner initialized successfully")

    def command_callback(self, msg: String):
        """Handle incoming natural language commands"""
        try:
            # Add command to processing queue
            command = msg.data
            task_future = asyncio.run_coroutine_threadsafe(
                self.task_queue.put(command),
                asyncio.get_event_loop()
            )

            self.get_logger().info(f"Received command: {command}")

        except Exception as e:
            self.get_logger().error(f"Error processing command: {str(e)}")

    async def process_task_queue(self):
        """Process tasks from the queue"""
        while rclpy.ok():
            try:
                # Get command from queue
                command = await asyncio.wait_for(
                    self.task_queue.get(),
                    timeout=1.0
                )

                # Process the command
                await self.process_command_async(command)

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in task queue processing: {str(e)}")

    async def process_command_async(self, command: str):
        """Process a command asynchronously"""
        try:
            # Get current context
            context = await self.get_current_context()

            # Plan the task using LLM
            task_plan = await self.plan_task_with_llm(command, context)

            if task_plan:
                # Execute the task plan
                await self.execute_task_plan(task_plan)
            else:
                self.get_logger().error(f"Failed to plan task for command: {command}")

        except Exception as e:
            self.get_logger().error(f"Error processing command {command}: {str(e)}")

    async def get_current_context(self) -> Dict[str, Any]:
        """Get current context for task planning"""
        # This would integrate with other ROS 2 nodes to get current state
        context = {
            'timestamp': time.time(),
            'robot_state': await self.get_robot_state(),
            'environment_state': await self.get_environment_state(),
            'recent_interactions': await self.get_recent_interactions()
        }

        return context

    async def plan_task_with_llm(self, command: str, context: Dict[str, Any]) -> Optional[TaskPlan]:
        """Plan task using LLM with context"""
        try:
            # Use context-aware planner
            task_plan = self.context_planner.plan_with_context(command, context)

            self.get_logger().info(f"Planned {len(task_plan.steps)} steps for command: {command}")

            return task_plan

        except Exception as e:
            self.get_logger().error(f"LLM planning failed: {str(e)}")
            return None

    async def execute_task_plan(self, task_plan: TaskPlan):
        """Execute a planned task sequence"""
        try:
            # Validate dependencies and execute steps
            execution_results = []

            for step in task_plan.steps:
                # Check dependencies
                if not self.check_dependencies_met(step, execution_results):
                    self.get_logger().warn(f"Dependencies not met for step {step.id}")
                    continue

                # Execute the step
                result = await self.execute_task_step(step)
                execution_results.append(result)

                # Publish status
                status_msg = String()
                status_msg.data = f"Step {step.id}: {result['status']}"
                self.task_status_pub.publish(status_msg)

                if not result['success']:
                    self.get_logger().error(f"Task step failed: {step.id}")
                    break

            # Publish final task status
            final_status = String()
            final_status.data = f"Task completed: {task_plan.original_command}"
            self.task_status_pub.publish(final_status)

        except Exception as e:
            self.get_logger().error(f"Task execution failed: {str(e)}")

    def check_dependencies_met(self, step: TaskStep, execution_results: List[Dict[str, Any]]) -> bool:
        """Check if all dependencies for a step are met"""
        for dep_id in step.dependencies:
            dep_result = next((r for r in execution_results if r['step_id'] == dep_id), None)
            if not dep_result or not dep_result['success']:
                return False
        return True

    async def execute_task_step(self, step: TaskStep) -> Dict[str, Any]:
        """Execute a single task step"""
        try:
            # Map task type to appropriate ROS 2 action
            if step.task_type == TaskType.NAVIGATION:
                result = await self.execute_navigation_step(step)
            elif step.task_type == TaskType.MANIPULATION:
                result = await self.execute_manipulation_step(step)
            elif step.task_type == TaskType.PERCEPTION:
                result = await self.execute_perception_step(step)
            else:
                result = await self.execute_generic_step(step)

            return result

        except Exception as e:
            return {
                'step_id': step.id,
                'success': False,
                'error': str(e),
                'status': 'failed'
            }

    async def execute_navigation_step(self, step: TaskStep) -> Dict[str, Any]:
        """Execute navigation task step"""
        # This would call navigation action server
        # For now, simulate execution
        await asyncio.sleep(2.0)  # Simulate navigation time

        return {
            'step_id': step.id,
            'success': True,
            'status': 'completed',
            'result': 'Navigation completed successfully'
        }

    async def execute_manipulation_step(self, step: TaskStep) -> Dict[str, Any]:
        """Execute manipulation task step"""
        # This would call manipulation action server
        # For now, simulate execution
        await asyncio.sleep(3.0)  # Simulate manipulation time

        return {
            'step_id': step.id,
            'success': True,
            'status': 'completed',
            'result': 'Manipulation completed successfully'
        }

    async def execute_perception_step(self, step: TaskStep) -> Dict[str, Any]:
        """Execute perception task step"""
        # This would call perception services
        # For now, simulate execution
        await asyncio.sleep(1.0)  # Simulate perception time

        return {
            'step_id': step.id,
            'success': True,
            'status': 'completed',
            'result': 'Perception completed successfully'
        }

    async def execute_generic_step(self, step: TaskStep) -> Dict[str, Any]:
        """Execute generic task step"""
        # For other task types, simulate execution
        await asyncio.sleep(1.0)

        return {
            'step_id': step.id,
            'success': True,
            'status': 'completed',
            'result': 'Task completed successfully'
        }
```

## Task Decomposition Algorithms

The effectiveness of cognitive planning depends on sophisticated task decomposition algorithms that can handle complex, multi-step commands.

### Hierarchical Task Networks (HTN)

Hierarchical Task Networks provide a structured approach to task decomposition:

```python
class HTNPlanner:
    """Hierarchical Task Network planner for complex task decomposition"""

    def __init__(self):
        self.primitive_tasks = {
            'navigate_to': self.navigate_to,
            'grasp_object': self.grasp_object,
            'place_object': self.place_object,
            'detect_object': self.detect_object,
            'open_gripper': self.open_gripper,
            'close_gripper': self.close_gripper
        }

        self.complex_tasks = {
            'clean_table': self.clean_table_method,
            'fetch_object': self.fetch_object_method,
            'set_table': self.set_table_method
        }

    def clean_table_method(self, context: Dict[str, Any]) -> List[TaskStep]:
        """Method to decompose 'clean table' task"""
        steps = []

        # 1. Navigate to table
        steps.append(TaskStep(
            id="clean_1",
            description="Navigate to the table",
            task_type=TaskType.NAVIGATION,
            ros_action="nav2_msgs/NavigateToPose",
            parameters={"target_pose": context.get('table_pose', [0.0, 0.0, 0.0])},
            dependencies=[],
            success_criteria="Robot reaches table location"
        ))

        # 2. Detect objects on table
        steps.append(TaskStep(
            id="clean_2",
            description="Detect objects on the table",
            task_type=TaskType.PERCEPTION,
            ros_action="sensor_msgs/Image",
            parameters={"sensor": "head_camera"},
            dependencies=["clean_1"],
            success_criteria="Objects identified on table"
        ))

        # 3. For each object, grasp and move away
        # This would be expanded based on detected objects
        steps.append(TaskStep(
            id="clean_3",
            description="Grasp detected object",
            task_type=TaskType.MANIPULATION,
            ros_action="manipulation_msgs/GraspObject",
            parameters={"object_id": "detected_object_1"},
            dependencies=["clean_2"],
            success_criteria="Object successfully grasped"
        ))

        return steps

    def fetch_object_method(self, context: Dict[str, Any]) -> List[TaskStep]:
        """Method to decompose 'fetch object' task"""
        steps = []

        # 1. Navigate to object location
        steps.append(TaskStep(
            id="fetch_1",
            description="Navigate to object location",
            task_type=TaskType.NAVIGATION,
            ros_action="nav2_msgs/NavigateToPose",
            parameters={"target_pose": context.get('object_pose', [0.0, 0.0, 0.0])},
            dependencies=[],
            success_criteria="Robot reaches object location"
        ))

        # 2. Grasp the object
        steps.append(TaskStep(
            id="fetch_2",
            description="Grasp the object",
            task_type=TaskType.MANIPULATION,
            ros_action="manipulation_msgs/GraspObject",
            parameters={"object_id": context.get('object_id', 'unknown')},
            dependencies=["fetch_1"],
            success_criteria="Object successfully grasped"
        ))

        # 3. Navigate back to user
        steps.append(TaskStep(
            id="fetch_3",
            description="Navigate back to user",
            task_type=TaskType.NAVIGATION,
            ros_action="nav2_msgs/NavigateToPose",
            parameters={"target_pose": context.get('user_pose', [0.0, 0.0, 0.0])},
            dependencies=["fetch_2"],
            success_criteria="Robot returns to user location"
        ))

        # 4. Place object for user
        steps.append(TaskStep(
            id="fetch_4",
            description="Place object for user",
            task_type=TaskType.MANIPULATION,
            ros_action="manipulation_msgs/PlaceObject",
            parameters={"placement_pose": context.get('delivery_pose', [0.0, 0.5, 0.8])},
            dependencies=["fetch_3"],
            success_criteria="Object placed for user access"
        ))

        return steps

    def navigate_to(self, params: Dict[str, Any]) -> bool:
        """Primitive navigation task"""
        # Implementation would call navigation action server
        return True

    def grasp_object(self, params: Dict[str, Any]) -> bool:
        """Primitive grasping task"""
        # Implementation would call manipulation action server
        return True

    def place_object(self, params: Dict[str, Any]) -> bool:
        """Primitive placing task"""
        # Implementation would call manipulation action server
        return True

    def detect_object(self, params: Dict[str, Any]) -> bool:
        """Primitive object detection task"""
        # Implementation would call perception service
        return True

    def open_gripper(self, params: Dict[str, Any]) -> bool:
        """Primitive gripper control task"""
        return True

    def close_gripper(self, params: Dict[str, Any]) -> bool:
        """Primitive gripper control task"""
        return True
```

## Safety and Error Handling in Cognitive Planning

Safety considerations are paramount in LLM-based cognitive planning systems, as incorrect task decomposition could lead to unsafe robot behavior.

### Safety Validation Layer

```python
class SafetyValidator:
    """Validates task plans for safety before execution"""

    def __init__(self, robot_state_provider, environment_provider):
        self.robot_state_provider = robot_state_provider
        self.environment_provider = environment_provider
        self.safety_rules = self.load_safety_rules()

    def load_safety_rules(self) -> Dict[str, Any]:
        """Load safety rules for task validation"""
        return {
            'navigation': {
                'max_speed': 0.5,  # m/s
                'min_obstacle_distance': 0.3,  # meters
                'no_go_zones': []
            },
            'manipulation': {
                'max_force': 50.0,  # Newtons
                'reachable_workspace': 'spherical',
                'fragile_objects': ['glass', 'ceramic', 'electronics']
            },
            'general': {
                'max_execution_time': 300.0,  # seconds
                'battery_threshold': 0.2,  # 20% minimum
                'emergency_stop': True
            }
        }

    def validate_task_plan(self, task_plan: TaskPlan) -> Tuple[bool, List[str]]:
        """Validate a task plan for safety compliance"""
        violations = []

        for step in task_plan.steps:
            step_violations = self.validate_task_step(step)
            violations.extend(step_violations)

        is_safe = len(violations) == 0
        return is_safe, violations

    def validate_task_step(self, step: TaskStep) -> List[str]:
        """Validate a single task step for safety"""
        violations = []

        # Validate based on task type
        if step.task_type == TaskType.NAVIGATION:
            violations.extend(self.validate_navigation_step(step))
        elif step.task_type == TaskType.MANIPULATION:
            violations.extend(self.validate_manipulation_step(step))
        elif step.task_type == TaskType.PERCEPTION:
            violations.extend(self.validate_perception_step(step))

        return violations

    def validate_navigation_step(self, step: TaskStep) -> List[str]:
        """Validate navigation task step"""
        violations = []

        # Check if target location is in no-go zone
        target_pose = step.parameters.get('target_pose', [0, 0, 0])

        for no_go_zone in self.safety_rules['navigation']['no_go_zones']:
            if self.is_in_no_go_zone(target_pose, no_go_zone):
                violations.append(f"Navigation target {target_pose} is in no-go zone")

        # Check for obstacles in path
        if not self.is_path_clear(target_pose):
            violations.append(f"Path to {target_pose} contains obstacles")

        return violations

    def validate_manipulation_step(self, step: TaskStep) -> List[str]:
        """Validate manipulation task step"""
        violations = []

        # Check if object is fragile and appropriate care is taken
        object_id = step.parameters.get('object_id', '')

        for fragile_obj in self.safety_rules['manipulation']['fragile_objects']:
            if fragile_obj.lower() in object_id.lower():
                # Check if gentle grasping is specified
                if not step.parameters.get('gentle_grasp', False):
                    violations.append(f"Fragile object {object_id} requires gentle grasping")

        # Check if target pose is in reachable workspace
        placement_pose = step.parameters.get('placement_pose')
        if placement_pose and not self.is_in_reachable_workspace(placement_pose):
            violations.append(f"Placement pose {placement_pose} is outside reachable workspace")

        return violations

    def validate_perception_step(self, step: TaskStep) -> List[str]:
        """Validate perception task step"""
        violations = []

        # Perception tasks generally have fewer safety constraints
        # but we might check for privacy considerations
        return violations

    def is_in_no_go_zone(self, pose: List[float], no_go_zone: Dict[str, Any]) -> bool:
        """Check if a pose is in a no-go zone"""
        # Implementation would check if pose is within no-go boundaries
        return False

    def is_path_clear(self, target_pose: List[float]) -> bool:
        """Check if path to target pose is clear of obstacles"""
        # Implementation would check navigation map for obstacles
        return True

    def is_in_reachable_workspace(self, pose: List[float]) -> bool:
        """Check if pose is in robot's reachable workspace"""
        # Implementation would check against robot's kinematic constraints
        return True
```

## Performance Optimization

### Caching and Memoization

To improve performance, the system implements caching for frequently requested task decompositions:

```python
import functools
import time
from typing import Callable

class TaskPlanCache:
    """Caches task plans to improve performance for repeated commands"""

    def __init__(self, max_size: int = 100, ttl: int = 300):  # 5 minutes TTL
        self.cache = {}
        self.max_size = max_size
        self.ttl = ttl

    def get(self, command: str, context_hash: str) -> Optional[TaskPlan]:
        """Get cached task plan if available and not expired"""
        cache_key = f"{command}_{context_hash}"

        if cache_key in self.cache:
            cached_item = self.cache[cache_key]
            if time.time() - cached_item['timestamp'] < self.ttl:
                return cached_item['task_plan']
            else:
                # Remove expired item
                del self.cache[cache_key]

        return None

    def put(self, command: str, context_hash: str, task_plan: TaskPlan):
        """Put task plan in cache"""
        cache_key = f"{command}_{context_hash}"

        # Remove oldest item if cache is full
        if len(self.cache) >= self.max_size:
            oldest_key = min(self.cache.keys(), key=lambda k: self.cache[k]['timestamp'])
            del self.cache[oldest_key]

        self.cache[cache_key] = {
            'task_plan': task_plan,
            'timestamp': time.time()
        }

    def clear(self):
        """Clear the cache"""
        self.cache.clear()
```

### Asynchronous Processing

The system uses asynchronous processing to handle multiple commands concurrently:

```python
import asyncio
from asyncio import Queue
import uuid

class AsyncTaskProcessor:
    """Handles asynchronous task processing with concurrency control"""

    def __init__(self, max_concurrent_tasks: int = 3):
        self.max_concurrent_tasks = max_concurrent_tasks
        self.task_queue = Queue()
        self.active_tasks = {}
        self.semaphore = asyncio.Semaphore(max_concurrent_tasks)

    async def submit_task(self, command: str, context: Dict[str, Any]) -> str:
        """Submit a task for processing"""
        task_id = str(uuid.uuid4())

        # Create task and add to queue
        task_coro = self.process_task(task_id, command, context)
        asyncio.create_task(task_coro)

        return task_id

    async def process_task(self, task_id: str, command: str, context: Dict[str, Any]):
        """Process a single task with concurrency control"""
        async with self.semaphore:
            self.active_tasks[task_id] = 'processing'

            try:
                # Process the task
                result = await self.execute_task_logic(command, context)
                self.active_tasks[task_id] = 'completed'

                return result
            except Exception as e:
                self.active_tasks[task_id] = 'failed'
                raise
            finally:
                if task_id in self.active_tasks:
                    del self.active_tasks[task_id]

    async def execute_task_logic(self, command: str, context: Dict[str, Any]):
        """Execute the core task logic"""
        # This would call the LLM task planner
        # Implementation would go here
        await asyncio.sleep(1)  # Simulate processing time
        return f"Processed: {command}"
```

## Integration with ROS 2 Ecosystem

The cognitive planning system integrates seamlessly with the broader ROS 2 ecosystem:

### Service Interfaces

```python
from rclpy.service import Service
from std_srvs.srv import Trigger
from humanoid_msgs.srv import PlanTask, ExecuteTask

class CognitivePlanningServices(Node):
    """Provides ROS 2 services for cognitive planning"""

    def __init__(self):
        super().__init__('cognitive_planning_services')

        # Task planning service
        self.plan_task_service = self.create_service(
            PlanTask,
            'plan_task',
            self.plan_task_callback
        )

        # Task execution service
        self.execute_task_service = self.create_service(
            ExecuteTask,
            'execute_task',
            self.execute_task_callback
        )

        # Task status service
        self.task_status_service = self.create_service(
            Trigger,
            'get_task_status',
            self.get_task_status_callback
        )

    def plan_task_callback(self, request: PlanTask.Request, response: PlanTask.Response):
        """Handle task planning request"""
        try:
            # Plan the task using LLM
            task_plan = self.llm_planner.process_command(request.command)

            # Convert to service response
            response.success = True
            response.message = "Task planned successfully"
            response.task_plan = self.task_plan_to_msg(task_plan)

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def execute_task_callback(self, request: ExecuteTask.Request, response: ExecuteTask.Response):
        """Handle task execution request"""
        try:
            # Execute the planned task
            execution_result = self.execute_task_plan(request.task_plan)

            response.success = execution_result['success']
            response.message = execution_result['message']

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def get_task_status_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle task status request"""
        try:
            status = self.get_current_task_status()
            response.success = True
            response.message = status
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def task_plan_to_msg(self, task_plan: TaskPlan):
        """Convert TaskPlan to ROS 2 message"""
        # Implementation to convert TaskPlan to appropriate ROS 2 message type
        pass

    def execute_task_plan(self, task_plan_msg):
        """Execute a task plan from ROS 2 message"""
        # Implementation to execute task plan
        pass

    def get_current_task_status(self) -> str:
        """Get current task execution status"""
        # Implementation to return current status
        return "idle"
```

## Future Developments and Research Directions

The field of LLM-based cognitive planning continues to evolve rapidly. Future developments include:

### Multimodal Integration

Future systems will incorporate vision, language, and action in a unified framework:

```python
class MultimodalPlanner:
    """Integrates vision, language, and action for enhanced cognitive planning"""

    def __init__(self):
        self.vision_processor = VisionProcessor()
        self.language_processor = LanguageProcessor()
        self.action_planner = ActionPlanner()

    def multimodal_plan(self, command: str, visual_input: Any) -> TaskPlan:
        """Plan tasks using both language and visual input"""
        # Process visual input
        visual_analysis = self.vision_processor.analyze(visual_input)

        # Process language command
        language_analysis = self.language_processor.analyze(command)

        # Combine analyses for planning
        combined_context = self.combine_analyses(visual_analysis, language_analysis)

        # Generate task plan
        task_plan = self.action_planner.plan(combined_context)

        return task_plan
```

### Continuous Learning

Systems will adapt and improve based on execution feedback:

```python
class ContinuousLearningPlanner:
    """Implements continuous learning from task execution outcomes"""

    def __init__(self):
        self.execution_history = []
        self.performance_metrics = {}

    def update_from_execution(self, command: str, task_plan: TaskPlan, outcome: Dict[str, Any]):
        """Update planning model based on execution outcome"""
        self.execution_history.append({
            'command': command,
            'task_plan': task_plan,
            'outcome': outcome,
            'timestamp': time.time()
        })

        # Update performance metrics
        self.update_metrics(outcome)

        # Potentially update planning strategies based on outcomes
        self.adapt_planning_strategies()

    def update_metrics(self, outcome: Dict[str, Any]):
        """Update performance metrics based on execution outcome"""
        # Implementation to track success rates, execution times, etc.
        pass

    def adapt_planning_strategies(self):
        """Adapt planning strategies based on historical performance"""
        # Implementation to adjust planning approaches
        pass
```

## Conclusion

LLM-based cognitive planning represents a significant advancement in robotic task execution, enabling natural language interaction and sophisticated task decomposition (Wei et al., 2022). The integration of GPT-4o with ROS 2 creates a powerful middleware layer that translates high-level human commands into executable robotic actions.

The systems described in this chapter provide a robust foundation for cognitive planning, incorporating safety validation, performance optimization, and seamless ROS 2 integration. As LLM technology continues to advance, these systems will become increasingly sophisticated, enabling more natural and intuitive human-robot collaboration.

The future of cognitive planning lies in multimodal integration, continuous learning, and enhanced safety systems that can adapt to diverse environments and user needs. These developments will further bridge the gap between human communication and robotic action, creating more capable and intuitive robotic systems.

## References

Achiam, J., Adler, S., Agarwal, S., Ahmad, S., Akbar, A., Almeida, D., ... & Zhang, C. (2023). GPT-4 technical report. *arXiv preprint arXiv:2303.08774*.

Brohan, A., Brown, J. A., Carbajal, J., Chebotar, Y., de Fender, S., Finn, C., ... & Zhu, Y. (2022). Language models as zero-shot planners: Extracting actionable knowledge for embodied agents. *Proceedings of the 39th International Conference on Machine Learning*, 2160-2174.

Huang, W., Abbeel, P., Pathak, D., & Mordatch, I. (2022). Language to rewards for robotic skill synthesis. *arXiv preprint arXiv:2202.02435*.

Patel, R., Baral, C., & Yang, Y. (2023). HuggingGPT: Solving AI tasks with ChatGPT and its friends in Hugging Face. *arXiv preprint arXiv:2303.17580*.

Wei, J., Wang, X., Schuurmans, D., Bosma, M., Ichter, B., Xia, F., ... & Zhou, D. (2022). Chain-of-thought prompting elicits reasoning in large language models. *Advances in Neural Information Processing Systems*, 35, 24824-24837.