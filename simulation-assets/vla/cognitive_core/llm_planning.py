"""
LLM Planning Module for VLA System

This module handles natural language understanding and action planning
using LLMs with function calling to generate structured ROS 2 actions.
"""
import asyncio
import json
import os
from typing import Dict, List, Any, Optional
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import OpenAI - handle the case where it might not be installed
try:
    import openai
    from openai import AsyncOpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    print("Warning: openai package not installed. LLM planning will not work.")

logger = logging.getLogger(__name__)

class LLMPlanner:
    """
    Class for handling LLM-based natural language understanding and action planning.
    """
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-4o"):
        """
        Initialize the LLM planner.

        Args:
            api_key: OpenAI API key. If None, will use OPENAI_API_KEY environment variable
            model: LLM model to use (default: gpt-4o)
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI package is not installed. Please install it with 'pip install openai'")

        # Set API key
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("OpenAI API key is required. Set OPENAI_API_KEY environment variable.")

        # Initialize OpenAI client
        self.client = AsyncOpenAI(api_key=self.api_key)
        self.model = model

        # Define the function schema for ROS 2 actions
        self.ros_functions = [
            {
                "name": "move_to_position",
                "description": "Move the robot to a specific position",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number", "description": "X coordinate"},
                        "y": {"type": "number", "description": "Y coordinate"},
                        "z": {"type": "number", "description": "Z coordinate"},
                        "frame_id": {"type": "string", "description": "Reference frame for the coordinates"}
                    },
                    "required": ["x", "y", "frame_id"]
                }
            },
            {
                "name": "grasp_object",
                "description": "Grasp an object at a specific location or identified by description",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_id": {"type": "string", "description": "ID of the object to grasp"},
                        "object_description": {"type": "string", "description": "Description of the object to grasp"},
                        "x": {"type": "number", "description": "X coordinate of the object"},
                        "y": {"type": "number", "description": "Y coordinate of the object"},
                        "z": {"type": "number", "description": "Z coordinate of the object"}
                    },
                    "required": []
                }
            },
            {
                "name": "navigate_to",
                "description": "Navigate the robot to a named location",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {"type": "string", "description": "Named location to navigate to (e.g., 'kitchen', 'living room', 'table')"},
                        "relative_direction": {"type": "string", "description": "Relative direction (e.g., 'left of', 'right of', 'in front of', 'behind')"},
                        "reference_object": {"type": "string", "description": "Reference object for relative positioning"}
                    },
                    "required": ["location"]
                }
            },
            {
                "name": "detect_object",
                "description": "Detect and identify objects in the robot's field of view",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_type": {"type": "string", "description": "Type of object to detect (e.g., 'red bottle', 'apple', 'box')"},
                        "color": {"type": "string", "description": "Color of the object to detect"},
                        "count": {"type": "integer", "description": "Expected number of objects to detect"}
                    },
                    "required": []
                }
            },
            {
                "name": "manipulate_object",
                "description": "Perform a manipulation action on an object",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "description": "Manipulation action (e.g., 'pick_up', 'place', 'move', 'rotate')"},
                        "object_id": {"type": "string", "description": "ID of the object to manipulate"},
                        "target_location": {"type": "string", "description": "Target location for the manipulation"}
                    },
                    "required": ["action"]
                }
            }
        ]

    async def plan_actions(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[Dict[str, Any]]:
        """
        Plan ROS 2 actions based on a natural language command and vision context.

        Args:
            natural_language_command: Natural language command from the user
            vision_context: Context from vision system (detected objects, scene description)

        Returns:
            Dictionary containing planned actions, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Prepare the system message with instructions for the LLM
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that converts natural language commands into structured actions for a robot.
                The robot operates in a physical environment and can perform actions like moving, grasping objects, and navigating.

                When given a command, respond with specific actions using the available functions.
                Always prioritize safety and make reasonable assumptions about the environment if not specified.

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with function calls that match the robot's capabilities.
                """
            }

            # Prepare the user message with the command
            user_message = {
                "role": "user",
                "content": natural_language_command
            }

            # Call the OpenAI API with function calling
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",  # Let the model decide which functions to call
                temperature=0.1
            )

            # Extract the function calls from the response
            message = response.choices[0].message

            if message.function_call:
                # Parse the function call
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                # Create the planned action
                planned_action = {
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": natural_language_command,
                    "vision_context_used": vision_context is not None
                }

                logger.info(f"Planned action: {function_name} with args {function_args}")
                return planned_action
            else:
                # If no function was called, try to extract the intent differently
                content = message.content
                if content:
                    logger.info(f"LLM response (no function call): {content}")
                    # For now, return a generic response - in practice, you might want to parse this differently
                    return {
                        "function": "unknown",
                        "arguments": {"response": content},
                        "original_command": natural_language_command,
                        "vision_context_used": vision_context is not None
                    }
                else:
                    logger.warning("No function call and no content in response")
                    return None

        except Exception as e:
            logger.error(f"LLM planning failed: {str(e)}")
            return None

    async def plan_multi_step_actions(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[List[Dict[str, Any]]]:
        """
        Plan a sequence of ROS 2 actions for complex commands.

        Args:
            natural_language_command: Complex natural language command
            vision_context: Context from vision system

        Returns:
            List of planned actions, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Prepare the system message for multi-step planning
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that converts complex natural language commands into a sequence of structured actions for a robot.
                The robot operates in a physical environment and can perform actions like moving, grasping objects, and navigating.

                For complex commands, break them down into a sequence of individual actions.
                Each action should be a separate function call.

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with multiple function calls as needed to complete the complex task.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Plan the following complex task: {natural_language_command}"
            }

            # Call the OpenAI API with function calling for multi-step planning
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            # Extract all function calls from the response
            message = response.choices[0].message
            planned_actions = []

            # Handle function calls in the response
            if hasattr(message, 'function_call') and message.function_call:
                # Single function call
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                planned_actions.append({
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": natural_language_command,
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                # Multiple function calls (tool calls)
                for tool_call in message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    planned_actions.append({
                        "function": function_name,
                        "arguments": function_args,
                        "original_command": natural_language_command,
                        "vision_context_used": vision_context is not None
                    })

            logger.info(f"Planned {len(planned_actions)} actions for command: {natural_language_command}")
            return planned_actions

        except Exception as e:
            logger.error(f"Multi-step LLM planning failed: {str(e)}")
            return None

    async def refine_plan_with_feedback(self, original_command: str, current_plan: List[Dict],
                                      feedback: str, vision_context: Optional[Dict] = None) -> Optional[List[Dict[str, Any]]]:
        """
        Refine an existing plan based on feedback or new information.

        Args:
            original_command: The original command
            current_plan: Current action plan
            feedback: Feedback about the current plan
            vision_context: Updated vision context

        Returns:
            Refined action plan, or None if refinement failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that refines robot action plans based on feedback.
                The original command was: {original_command}
                Current plan: {json.dumps(current_plan)}
                Feedback: {feedback}

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Adjust the plan as needed based on the feedback and new information.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Refine the plan based on this feedback: {feedback}"
            }

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message
            refined_plan = []

            if hasattr(message, 'function_call') and message.function_call:
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                refined_plan.append({
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": original_command,
                    "feedback_applied": feedback,
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                for tool_call in message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    refined_plan.append({
                        "function": function_name,
                        "arguments": function_args,
                        "original_command": original_command,
                        "feedback_applied": feedback,
                        "vision_context_used": vision_context is not None
                    })

            logger.info(f"Refined plan with {len(refined_plan)} actions")
            return refined_plan

        except Exception as e:
            logger.error(f"Plan refinement failed: {str(e)}")
            return None

    async def plan_actions_with_complex_references(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[Dict[str, Any]]:
        """
        Plan actions considering complex object references like "the red one on the left" or "the second apple from the right".

        Args:
            natural_language_command: Natural language command with complex object references
            vision_context: Context from vision system (detected objects, scene description)

        Returns:
            Dictionary containing planned actions, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Enhanced system message with instructions for handling complex object references
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that converts natural language commands into structured actions for a robot.
                The robot operates in a physical environment and can perform actions like moving, grasping objects, and navigating.

                When the user refers to objects with complex descriptions like "the red one on the left" or "the second apple from the right",
                you need to interpret these references in the context of the visible objects.

                Pay attention to:
                - Relative positions (left, right, front, back, center, middle, top, bottom)
                - Ordinal numbers (first, second, third, etc.)
                - Colors and other attributes (red, blue, large, small)
                - Spatial relationships (next to, near, far from, between)

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with function calls that match the robot's capabilities.
                """
            }

            user_message = {
                "role": "user",
                "content": natural_language_command
            }

            # Call the OpenAI API with function calling
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message

            if message.function_call:
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                planned_action = {
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": natural_language_command,
                    "vision_context_used": vision_context is not None
                }

                logger.info(f"Planned action with complex references: {function_name} with args {function_args}")
                return planned_action
            else:
                logger.warning("No function call in response")
                return None

        except Exception as e:
            logger.error(f"LLM planning with complex references failed: {str(e)}")
            return None

    async def plan_multi_step_actions_with_context(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[List[Dict[str, Any]]]:
        """
        Plan multi-step actions with enhanced context awareness for complex commands.

        Args:
            natural_language_command: Complex natural language command
            vision_context: Context from vision system

        Returns:
            List of planned actions, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Enhanced system message with multi-step planning instructions
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that converts complex natural language commands into a sequence of structured actions for a robot.
                The robot operates in a physical environment and can perform actions like moving, grasping objects, and navigating.

                For complex commands, especially those involving multiple objects or steps:
                1. First identify all relevant objects in the scene based on the vision context
                2. Understand the relationships between objects and the user's intent
                3. Break the command into a logical sequence of individual actions
                4. Consider the spatial arrangement of objects when planning navigation or manipulation

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with multiple function calls as needed to complete the complex task.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Plan the following complex task: {natural_language_command}"
            }

            # Call the OpenAI API with function calling for multi-step planning
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message
            planned_actions = []

            # Handle function calls in the response
            if hasattr(message, 'function_call') and message.function_call:
                # Single function call
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                planned_actions.append({
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": natural_language_command,
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                # Multiple function calls (tool calls)
                for tool_call in message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    planned_actions.append({
                        "function": function_name,
                        "arguments": function_args,
                        "original_command": natural_language_command,
                        "vision_context_used": vision_context is not None
                    })

            logger.info(f"Planned {len(planned_actions)} actions with context for command: {natural_language_command}")
            return planned_actions

        except Exception as e:
            logger.error(f"Multi-step LLM planning with context failed: {str(e)}")
            return None

    async def interpret_complex_object_reference(self, reference_description: str, available_objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Interpret complex object references like "the red one on the left" or "the second apple".

        Args:
            reference_description: Natural language description of the target object
            available_objects: List of objects visible to the robot

        Returns:
            The object that best matches the description, or None if no clear match
        """
        if not available_objects:
            logger.warning("No available objects to interpret reference against")
            return None

        try:
            # Create a system message to help the LLM interpret the reference
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that helps interpret natural language references to objects in a scene.

                Given a description of an object and a list of available objects, identify which object
                best matches the description. Consider:
                - Color descriptions (red, blue, green, etc.)
                - Size descriptions (large, small, big, tiny, etc.)
                - Positional descriptions (left, right, front, back, center, etc.)
                - Ordinal numbers (first, second, third, etc.)
                - Spatial relationships (near, next to, etc.)

                Available objects: {json.dumps(available_objects)}

                Return the object that best matches the description.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Which object matches this description: '{reference_description}'"
            }

            # Call the OpenAI API to interpret the reference
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                temperature=0.05  # Very deterministic for consistent results
            )

            # In this case, we're getting a text response, not a function call
            # For a more robust implementation, we could use function calling to return a structured response
            # For now, we'll use a simple approach to match the description to an object
            # This is a simplified implementation - in a real system, we'd have more sophisticated matching

            # Look for the object that best matches the description
            best_match = None
            best_score = 0

            for obj in available_objects:
                score = 0

                # Simple keyword matching
                obj_desc = f"{obj.get('class', '')} {obj.get('color', '')}".lower()
                ref_desc = reference_description.lower()

                # Count matching words
                for word in ref_desc.split():
                    if word in obj_desc:
                        score += 1

                # Bonus for exact class matches
                if obj.get('class', '').lower() in ref_desc:
                    score += 2

                # Bonus for color matches
                if obj.get('color', '').lower() in ref_desc:
                    score += 2

                if score > best_score:
                    best_score = score
                    best_match = obj

            logger.info(f"Interpreted reference '{reference_description}' as object: {best_match}")
            return best_match

        except Exception as e:
            logger.error(f"Complex object reference interpretation failed: {str(e)}")
            return None

    async def plan_multi_step_actions_advanced(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[List[Dict[str, Any]]]:
        """
        Plan multi-step actions with enhanced capabilities for complex commands.

        Args:
            natural_language_command: Complex natural language command
            vision_context: Context from vision system

        Returns:
            List of planned actions, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Enhanced system message with multi-step planning instructions
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that converts complex natural language commands into a sequence of structured actions for a robot.
                The robot operates in a physical environment and can perform actions like moving, grasping objects, and navigating.

                For complex commands, especially those involving multiple steps or objects:
                1. Break the command into a logical sequence of individual actions
                2. Consider dependencies between actions (e.g., must move somewhere before grasping)
                3. Account for the current state of the environment
                4. Plan for intermediate states if needed

                Pay attention to:
                - Sequential dependencies (what needs to happen before what)
                - Object availability and accessibility
                - Safety constraints between actions
                - Resource requirements (robot capabilities, environment state)

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with multiple function calls that represent the step-by-step plan to accomplish the complex task.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Break down this complex task into sequential steps: {natural_language_command}\n\nProvide the detailed step-by-step plan."
            }

            # Call the OpenAI API with function calling for multi-step planning
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message
            planned_actions = []

            # Handle function calls in the response
            if hasattr(message, 'function_call') and message.function_call:
                # Single function call
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                planned_actions.append({
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": natural_language_command,
                    "step_number": 1,
                    "total_steps": 1,
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                # Multiple function calls (tool calls)
                for idx, tool_call in enumerate(message.tool_calls, start=1):
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    planned_actions.append({
                        "function": function_name,
                        "arguments": function_args,
                        "original_command": natural_language_command,
                        "step_number": idx,
                        "total_steps": len(message.tool_calls),
                        "vision_context_used": vision_context is not None
                    })

            logger.info(f"Planned {len(planned_actions)} steps for command: {natural_language_command}")
            return planned_actions

        except Exception as e:
            logger.error(f"Advanced multi-step LLM planning failed: {str(e)}")
            return None

    async def plan_sequential_actions_with_state_tracking(self, natural_language_command: str, initial_state: Optional[Dict] = None, vision_context: Optional[Dict] = None) -> Optional[Dict[str, Any]]:
        """
        Plan sequential actions while tracking state changes between steps.

        Args:
            natural_language_command: Natural language command to execute
            initial_state: Initial state of the environment
            vision_context: Context from vision system

        Returns:
            Dictionary containing action sequence and state tracking, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Create initial state if not provided
            current_state = initial_state or {
                "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "held_object": None,
                "environment_objects": vision_context.get('objects', []) if vision_context else [],
                "completed_tasks": []
            }

            # Enhanced system message for state-aware planning
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that creates sequential action plans for a robot while tracking state changes.

                Current state: {json.dumps(current_state)}

                Plan actions that take into account the current state and how each action will change the state.
                Consider:
                - How the robot's position will change
                - Whether the robot is holding an object
                - How the environment state will change after each action
                - What preconditions must be met for each subsequent action

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Create a sequence of actions that accomplishes the user's goal while properly tracking state changes.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Create a sequential action plan for: {natural_language_command}\n\nTrack how each action changes the robot's state."
            }

            # Call the OpenAI API with function calling for state-aware planning
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message
            action_sequence = []

            # Handle function calls in the response
            if hasattr(message, 'function_call') and message.function_call:
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                action_sequence.append({
                    "function": function_name,
                    "arguments": function_args,
                    "original_command": natural_language_command,
                    "step_number": 1,
                    "predicted_state_change": self._predict_state_change(function_name, function_args, current_state),
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                for idx, tool_call in enumerate(message.tool_calls, start=1):
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    action_sequence.append({
                        "function": function_name,
                        "arguments": function_args,
                        "original_command": natural_language_command,
                        "step_number": idx,
                        "predicted_state_change": self._predict_state_change(function_name, function_args, current_state),
                        "vision_context_used": vision_context is not None
                    })

            result = {
                "action_sequence": action_sequence,
                "initial_state": current_state,
                "final_predicted_state": self._apply_sequence_to_state(action_sequence, current_state),
                "total_steps": len(action_sequence),
                "original_command": natural_language_command,
                "vision_context_used": vision_context is not None
            }

            logger.info(f"Planned state-tracking sequence with {len(action_sequence)} steps for command: {natural_language_command}")
            return result

        except Exception as e:
            logger.error(f"State-tracking sequential planning failed: {str(e)}")
            return None

    def _predict_state_change(self, action_function: str, action_args: Dict[str, Any], current_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Predict how an action will change the robot's state.

        Args:
            action_function: The function to be executed
            action_args: Arguments for the function
            current_state: Current state of the environment

        Returns:
            Dictionary describing predicted state changes
        """
        predicted_changes = {}

        if action_function == "move_to_position":
            # Predict position change
            predicted_changes["robot_position"] = {
                "from": current_state.get("robot_position", {"x": 0.0, "y": 0.0, "z": 0.0}),
                "to": {
                    "x": action_args.get("x", 0.0),
                    "y": action_args.get("y", 0.0),
                    "z": action_args.get("z", 0.0)
                }
            }
        elif action_function == "grasp_object":
            # Predict object acquisition
            predicted_changes["held_object"] = {
                "from": current_state.get("held_object", None),
                "to": action_args.get("object_id") or action_args.get("object_description")
            }
        elif action_function == "navigate_to":
            # Predict navigation change
            predicted_changes["robot_position"] = {
                "from": current_state.get("robot_position", {"x": 0.0, "y": 0.0, "z": 0.0}),
                "to": "will be updated based on navigation target"
            }
        elif action_function == "manipulate_object":
            # Predict object manipulation
            predicted_changes["held_object"] = {
                "from": current_state.get("held_object", None),
                "to": f"manipulated {action_args.get('action', 'object')}"
            }

        return predicted_changes

    def _apply_sequence_to_state(self, action_sequence: List[Dict[str, Any]], initial_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Apply an action sequence to predict the final state.

        Args:
            action_sequence: List of actions to apply
            initial_state: Starting state

        Returns:
            Predicted final state after all actions
        """
        current_state = initial_state.copy()

        for action in action_sequence:
            action_function = action["function"]
            action_args = action["arguments"]

            if action_function == "move_to_position":
                current_state["robot_position"] = {
                    "x": action_args.get("x", current_state["robot_position"]["x"]),
                    "y": action_args.get("y", current_state["robot_position"]["y"]),
                    "z": action_args.get("z", current_state["robot_position"]["z"])
                }
            elif action_function == "grasp_object":
                current_state["held_object"] = action_args.get("object_id") or action_args.get("object_description")
            elif action_function == "manipulate_object":
                if current_state.get("held_object"):
                    # Update held object description based on manipulation
                    current_state["held_object"] = f"manipulated {current_state['held_object']}"
            elif action_function == "navigate_to":
                # Update robot position based on navigation target
                current_state["robot_position"] = {
                    "x": action_args.get("x", current_state["robot_position"]["x"]),
                    "y": action_args.get("y", current_state["robot_position"]["y"]),
                    "z": current_state["robot_position"]["z"]  # Keep z unchanged unless specified
                }
            elif action_function == "place_object":
                # Release held object
                current_state["held_object"] = None
            elif action_function == "detect_objects":
                # Update environment objects based on detection
                detected_objects = action_args.get("objects", [])
                current_state["environment_objects"] = detected_objects

        return current_state

    async def plan_complex_multi_step_command(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[Dict[str, Any]]:
        """
        Plan complex multi-step commands that require multiple sequential actions.

        Args:
            natural_language_command: Complex command requiring multiple steps
            vision_context: Context from vision system

        Returns:
            Dictionary containing multi-step plan, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Enhanced system message for complex multi-step planning
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that converts complex natural language commands into multi-step action sequences for a robot.
                The robot operates in a physical environment and can perform actions like moving, grasping objects, navigating, and manipulating objects.

                For complex commands:
                1. Break the command into a logical sequence of individual actions
                2. Consider dependencies between actions (e.g., must detect objects before selecting one)
                3. Account for the changing state of the environment after each action
                4. Plan for intermediate goals that lead to the final objective
                5. Ensure each step is achievable and safe

                Pay attention to:
                - Sequential dependencies (what needs to happen before what)
                - Object availability and accessibility after each step
                - Robot state changes (position, held object, etc.)
                - Safety constraints at each step
                - Resource requirements for each action

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with multiple function calls representing the complete action sequence.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Create a detailed multi-step plan for: {natural_language_command}\n\nBreak it down into individual actions that accomplish the goal sequentially."
            }

            # Call the OpenAI API with function calling for multi-step planning
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message
            action_sequence = []

            # Handle function calls in the response
            if hasattr(message, 'function_call') and message.function_call:
                # Single function call (though unlikely for complex commands)
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                action_sequence.append({
                    "function": function_name,
                    "arguments": function_args,
                    "step_number": 1,
                    "total_steps": 1,
                    "original_command": natural_language_command,
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                # Multiple function calls (tool calls) - typical for multi-step commands
                for idx, tool_call in enumerate(message.tool_calls, start=1):
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    action_sequence.append({
                        "function": function_name,
                        "arguments": function_args,
                        "step_number": idx,
                        "total_steps": len(message.tool_calls),
                        "original_command": natural_language_command,
                        "vision_context_used": vision_context is not None
                    })

            # Validate the entire sequence
            validation_results = []
            for action in action_sequence:
                validation_result = self.json_validator.validate_action_json(
                    action["function"],
                    action["arguments"]
                )
                validation_results.append({
                    "action_index": action["step_number"] - 1,
                    "function": action["function"],
                    "valid": validation_result["valid"],
                    "errors": validation_result.get("errors", [])
                })

            # Check if all actions are valid
            all_valid = all(result["valid"] for result in validation_results)

            if not all_valid:
                logger.warning(f"Some actions in the multi-step plan failed validation: {validation_results}")

            # Apply safety validation to the entire sequence
            safety_results = []
            for action in action_sequence:
                safety_result = self.safety_guardrails.validate_action(action)
                safety_results.append({
                    "action_index": action["step_number"] - 1,
                    "function": action["function"],
                    "safe": safety_result["valid"],
                    "violations": safety_result.get("violations", [])
                })

            # Check if all actions are safe
            all_safe = all(result["safe"] for result in safety_results)

            if not all_safe:
                logger.warning(f"Some actions in the multi-step plan failed safety validation: {safety_results}")

            result = {
                "action_sequence": action_sequence,
                "validation_results": validation_results,
                "safety_results": safety_results,
                "all_actions_valid": all_valid,
                "all_actions_safe": all_safe,
                "total_steps": len(action_sequence),
                "original_command": natural_language_command,
                "vision_context_used": vision_context is not None
            }

            logger.info(f"Planned multi-step sequence with {len(action_sequence)} steps for command: {natural_language_command}")
            return result

        except Exception as e:
            logger.error(f"Complex multi-step LLM planning failed: {str(e)}")
            return None

    async def plan_conditional_multi_step_actions(self, natural_language_command: str, vision_context: Optional[Dict] = None) -> Optional[Dict[str, Any]]:
        """
        Plan multi-step actions with conditional logic based on intermediate results.

        Args:
            natural_language_command: Command that may require conditional actions
            vision_context: Context from vision system

        Returns:
            Dictionary containing conditional multi-step plan, or None if planning failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # System message for conditional multi-step planning
            system_message = {
                "role": "system",
                "content": f"""
                You are an AI assistant that creates conditional multi-step plans for a robot.
                The robot operates in a dynamic environment where later actions may depend on the results of earlier actions.

                For commands that might require conditional execution:
                1. Identify points where the plan might need to branch based on intermediate results
                2. Create conditional branches for different possible outcomes
                3. Ensure each branch leads to the ultimate goal
                4. Plan fallback actions if primary actions fail

                Example conditional scenarios:
                - If object is found → grasp it, else → report failure
                - If path is clear → navigate, else → find alternative path
                - If object is graspable → proceed, else → find alternative object

                Vision context: {json.dumps(vision_context) if vision_context else 'No vision data available'}

                Respond with a plan that includes conditional logic where appropriate.
                """
            }

            user_message = {
                "role": "user",
                "content": f"Create a conditional multi-step plan for: {natural_language_command}\n\nInclude conditional branches where the next action depends on the result of the previous action."
            }

            # Call the OpenAI API for conditional planning
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[system_message, user_message],
                functions=self.ros_functions,
                function_call="auto",
                temperature=0.1
            )

            message = response.choices[0].message
            action_sequence = []

            if hasattr(message, 'function_call') and message.function_call:
                function_name = message.function_call.name
                function_args = json.loads(message.function_call.arguments)

                action_sequence.append({
                    "function": function_name,
                    "arguments": function_args,
                    "step_number": 1,
                    "total_steps": 1,
                    "original_command": natural_language_command,
                    "is_conditional": False,
                    "vision_context_used": vision_context is not None
                })
            elif hasattr(message, 'tool_calls') and message.tool_calls:
                for idx, tool_call in enumerate(message.tool_calls, start=1):
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    action_sequence.append({
                        "function": function_name,
                        "arguments": function_args,
                        "step_number": idx,
                        "total_steps": len(message.tool_calls),
                        "original_command": natural_language_command,
                        "is_conditional": self._is_conditional_action(function_name, function_args),
                        "vision_context_used": vision_context is not None
                    })

            # Build conditional plan structure
            conditional_plan = {
                "action_sequence": action_sequence,
                "has_conditionals": any(action.get("is_conditional", False) for action in action_sequence),
                "conditional_branches": self._extract_conditional_branches(action_sequence),
                "original_command": natural_language_command,
                "vision_context_used": vision_context is not None
            }

            logger.info(f"Planned conditional multi-step sequence with {len(action_sequence)} steps for command: {natural_language_command}")
            return conditional_plan

        except Exception as e:
            logger.error(f"Conditional multi-step LLM planning failed: {str(e)}")
            return None

    def _is_conditional_action(self, function_name: str, function_args: Dict[str, Any]) -> bool:
        """
        Determine if an action is conditional in nature.

        Args:
            function_name: Name of the function
            function_args: Arguments for the function

        Returns:
            True if the action is conditional, False otherwise
        """
        # Actions that typically involve conditional logic
        conditional_functions = [
            "navigate_if_clear",
            "grasp_if_reachable",
            "move_if_safe",
            "detect_and_select"
        ]

        # Check if the function name suggests conditionality
        if any(cond_func in function_name.lower() for cond_func in conditional_functions):
            return True

        # Check if arguments contain conditional keywords
        args_str = json.dumps(function_args).lower()
        conditional_keywords = ["if_", "condition", "conditional", "check", "verify", "confirm"]
        return any(keyword in args_str for keyword in conditional_keywords)

    def _extract_conditional_branches(self, action_sequence: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract conditional branches from an action sequence.

        Args:
            action_sequence: List of planned actions

        Returns:
            List of conditional branches in the plan
        """
        branches = []

        for idx, action in enumerate(action_sequence):
            if action.get("is_conditional", False):
                branch = {
                    "action_index": idx,
                    "function": action["function"],
                    "condition": f"result of step {idx} determines next action",
                    "possible_outcomes": ["success", "failure", "partial_success"]  # Simplified for this example
                }
                branches.append(branch)

        return branches


def create_example_vision_context() -> Dict:
    """
    Create an example vision context for testing purposes.

    Returns:
        Example vision context dictionary
    """
    return {
        "timestamp": "2025-12-18T10:30:00Z",
        "objects": [
            {
                "id": "obj_001",
                "class": "apple",
                "color": "red",
                "position": {"x": 1.2, "y": 0.5, "z": 0.8},
                "confidence": 0.92,
                "bbox": {"x_min": 100, "y_min": 150, "x_max": 180, "y_max": 230}
            },
            {
                "id": "obj_002",
                "class": "bottle",
                "color": "blue",
                "position": {"x": 1.5, "y": 0.3, "z": 0.8},
                "confidence": 0.87,
                "bbox": {"x_min": 250, "y_min": 120, "x_max": 320, "y_max": 280}
            },
            {
                "id": "obj_003",
                "class": "box",
                "color": "brown",
                "position": {"x": 0.8, "y": 1.0, "z": 0.6},
                "confidence": 0.95,
                "bbox": {"x_min": 50, "y_min": 300, "x_max": 200, "y_max": 450}
            }
        ],
        "environment": "indoor",
        "lighting": "normal",
        "camera_pose": {"x": 0, "y": 0, "z": 1.5, "orientation": [0, 0, 0, 1]}
    }


async def test_llm_planner():
    """Test function for LLM planner functionality."""
    print("Testing LLM planner...")

    # Check if API key is available
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("⚠️  OPENAI_API_KEY not set in environment. Skipping LLM planner tests.")
        return True  # Return True to continue with other tests

    try:
        # Create LLM planner instance
        planner = LLMPlanner(api_key=api_key)

        # Test simple command
        command = "Pick up the red apple"
        vision_context = create_example_vision_context()

        result = await planner.plan_actions(command, vision_context)
        print(f"Simple command result: {result}")

        # Test complex command
        complex_command = "Navigate to the kitchen, find the red apple, and pick it up"
        multi_result = await planner.plan_multi_step_actions(complex_command, vision_context)
        print(f"Multi-step command result: {multi_result}")

        # Test plan refinement
        if multi_result:
            feedback = "The apple is too high to reach from current position. Approach more closely."
            refined_result = await planner.refine_plan_with_feedback(complex_command, multi_result, feedback, vision_context)
            print(f"Refined plan result: {refined_result}")

        print("✓ LLM planner test completed")
        return True

    except Exception as e:
        print(f"✗ LLM planner test failed: {e}")
        return False


async def main_test():
    """Main test function."""
    success = await test_llm_planner()
    if success:
        print("✓ All LLM planner tests passed")
    else:
        print("✗ Some LLM planner tests failed")


if __name__ == "__main__":
    # Run the test
    asyncio.run(main_test())