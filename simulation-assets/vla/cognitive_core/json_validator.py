"""
JSON Validator Module for VLA System

This module validates JSON outputs from the LLM to ensure they conform
to expected ROS 2 action schemas and safety requirements.
"""
import json
from typing import Dict, Any, List, Optional, Union
import logging
from jsonschema import validate, ValidationError, Draft7Validator
import re

logger = logging.getLogger(__name__)

class JSONValidator:
    """
    Class for validating JSON outputs from LLM against ROS 2 action schemas.
    """
    def __init__(self):
        """
        Initialize the JSON validator with ROS 2 action schemas.
        """
        # Define schemas for different ROS 2 actions
        self.ros_action_schemas = {
            "move_to_position": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number", "default": 0.0},
                    "frame_id": {"type": "string", "default": "map"}
                },
                "required": ["x", "y"],
                "additionalProperties": False
            },
            "grasp_object": {
                "type": "object",
                "properties": {
                    "object_id": {"type": "string"},
                    "object_description": {"type": "string"},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"}
                },
                "oneOf": [
                    {"required": ["object_id"]},
                    {"required": ["object_description"]},
                    {"required": ["x", "y", "z"]}
                ],
                "additionalProperties": False
            },
            "navigate_to": {
                "type": "object",
                "properties": {
                    "location": {"type": "string"},
                    "relative_direction": {"type": "string", "enum": ["left of", "right of", "in front of", "behind"]},
                    "reference_object": {"type": "string"}
                },
                "required": ["location"],
                "additionalProperties": False
            },
            "detect_object": {
                "type": "object",
                "properties": {
                    "object_type": {"type": "string"},
                    "color": {"type": "string"},
                    "count": {"type": "integer", "minimum": 1}
                },
                "additionalProperties": False
            },
            "manipulate_object": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["pick_up", "place", "move", "rotate", "lift", "lower"]},
                    "object_id": {"type": "string"},
                    "target_location": {"type": "string"}
                },
                "required": ["action"],
                "additionalProperties": False
            }
        }

        # Define safety constraints
        self.safety_constraints = {
            "position_limits": {
                "x": {"min": -10.0, "max": 10.0},
                "y": {"min": -10.0, "max": 10.0},
                "z": {"min": 0.0, "max": 2.0}
            },
            "forbidden_actions": [
                "harm",
                "damage",
                "injure",
                "dangerous_command"
            ],
            "coordinate_precision": 3  # Maximum decimal places for coordinates
        }

    def validate_action_json(self, action_name: str, action_args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate an action's arguments against its schema.

        Args:
            action_name: Name of the action (e.g., "move_to_position")
            action_args: Arguments for the action

        Returns:
            Dictionary with validation results
        """
        if action_name not in self.ros_action_schemas:
            return {
                "valid": False,
                "errors": [f"Unknown action: {action_name}"],
                "action_name": action_name,
                "action_args": action_args
            }

        schema = self.ros_action_schemas[action_name]

        try:
            # Validate against schema
            validate(instance=action_args, schema=schema)

            # Perform additional safety checks
            safety_result = self._check_safety_constraints(action_name, action_args)

            if safety_result["valid"]:
                return {
                    "valid": True,
                    "errors": [],
                    "action_name": action_name,
                    "action_args": action_args,
                    "safety_check_passed": True
                }
            else:
                return {
                    "valid": False,
                    "errors": safety_result["errors"],
                    "action_name": action_name,
                    "action_args": action_args,
                    "safety_check_passed": False
                }

        except ValidationError as e:
            return {
                "valid": False,
                "errors": [f"Schema validation failed: {e.message}"],
                "action_name": action_name,
                "action_args": action_args,
                "safety_check_passed": False
            }
        except Exception as e:
            return {
                "valid": False,
                "errors": [f"Validation error: {str(e)}"],
                "action_name": action_name,
                "action_args": action_args,
                "safety_check_passed": False
            }

    def _check_safety_constraints(self, action_name: str, action_args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check action arguments against safety constraints.

        Args:
            action_name: Name of the action
            action_args: Arguments for the action

        Returns:
            Dictionary with safety validation results
        """
        errors = []

        # Check position limits for movement actions
        if action_name in ["move_to_position", "grasp_object"]:
            for coord in ["x", "y", "z"]:
                if coord in action_args:
                    value = action_args[coord]
                    limits = self.safety_constraints["position_limits"][coord]

                    if not (limits["min"] <= value <= limits["max"]):
                        errors.append(
                            f"Coordinate {coord}={value} is outside safe limits [{limits['min']}, {limits['max']}]"
                        )

        # Check for forbidden actions in descriptions
        for arg_name, arg_value in action_args.items():
            if isinstance(arg_value, str):
                for forbidden in self.safety_constraints["forbidden_actions"]:
                    if forbidden.lower() in arg_value.lower():
                        errors.append(f"Action contains forbidden term: {forbidden}")

        # Check coordinate precision
        for coord in ["x", "y", "z"]:
            if coord in action_args:
                value = action_args[coord]
                if isinstance(value, float):
                    decimal_places = len(str(value).split('.')[-1]) if '.' in str(value) else 0
                    if decimal_places > self.safety_constraints["coordinate_precision"]:
                        errors.append(
                            f"Coordinate {coord} has {decimal_places} decimal places, "
                            f"exceeding maximum of {self.safety_constraints['coordinate_precision']}"
                        )

        return {
            "valid": len(errors) == 0,
            "errors": errors
        }

    def validate_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate a sequence of actions.

        Args:
            action_sequence: List of action dictionaries

        Returns:
            Dictionary with validation results for the entire sequence
        """
        sequence_valid = True
        all_errors = []
        validated_actions = []

        for i, action in enumerate(action_sequence):
            if "function" not in action or "arguments" not in action:
                error = f"Action at index {i} missing required fields 'function' or 'arguments'"
                all_errors.append(error)
                sequence_valid = False
                continue

            action_name = action["function"]
            action_args = action["arguments"]

            result = self.validate_action_json(action_name, action_args)

            if not result["valid"]:
                all_errors.extend([f"Action {i}: {error}" for error in result["errors"]])
                sequence_valid = False

            validated_actions.append(result)

        return {
            "valid": sequence_valid,
            "errors": all_errors,
            "validated_actions": validated_actions,
            "total_actions": len(action_sequence),
            "valid_actions": sum(1 for r in validated_actions if r["valid"])
        }

    def validate_json_string(self, json_string: str) -> Dict[str, Any]:
        """
        Validate a JSON string from LLM output.

        Args:
            json_string: JSON string to validate

        Returns:
            Dictionary with validation results
        """
        try:
            # Parse the JSON string
            parsed_data = json.loads(json_string)

            # Check if it's a single action or sequence of actions
            if isinstance(parsed_data, dict) and "function" in parsed_data and "arguments" in parsed_data:
                # Single action
                result = self.validate_action_json(parsed_data["function"], parsed_data["arguments"])
                return {
                    "valid": result["valid"],
                    "errors": result["errors"],
                    "is_sequence": False,
                    "parsed_data": parsed_data
                }
            elif isinstance(parsed_data, list):
                # Sequence of actions
                result = self.validate_action_sequence(parsed_data)
                return {
                    "valid": result["valid"],
                    "errors": result["errors"],
                    "is_sequence": True,
                    "parsed_data": parsed_data,
                    "sequence_validation": result
                }
            else:
                return {
                    "valid": False,
                    "errors": ["JSON does not match expected action format"],
                    "is_sequence": False,
                    "parsed_data": parsed_data
                }

        except json.JSONDecodeError as e:
            return {
                "valid": False,
                "errors": [f"Invalid JSON format: {str(e)}"],
                "is_sequence": False,
                "parsed_data": None
            }
        except Exception as e:
            return {
                "valid": False,
                "errors": [f"Validation error: {str(e)}"],
                "is_sequence": False,
                "parsed_data": None
            }

    def sanitize_coordinates(self, action_args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Sanitize coordinate values to meet precision requirements.

        Args:
            action_args: Action arguments that may contain coordinates

        Returns:
            Sanitized action arguments
        """
        sanitized = action_args.copy()

        for coord in ["x", "y", "z"]:
            if coord in sanitized and isinstance(sanitized[coord], float):
                # Round to the required precision
                precision = self.safety_constraints["coordinate_precision"]
                sanitized[coord] = round(sanitized[coord], precision)

        return sanitized

    def validate_and_sanitize_action(self, action_name: str, action_args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate and sanitize an action in one step.

        Args:
            action_name: Name of the action
            action_args: Arguments for the action

        Returns:
            Dictionary with validation and sanitization results
        """
        # First, validate the action
        validation_result = self.validate_action_json(action_name, action_args)

        if validation_result["valid"]:
            # If valid, sanitize the coordinates
            sanitized_args = self.sanitize_coordinates(action_args)
            validation_result["sanitized_arguments"] = sanitized_args

        return validation_result

    def create_safe_action(self, action_name: str, action_args: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a safe version of an action by applying constraints.

        Args:
            action_name: Name of the action
            action_args: Arguments for the action

        Returns:
            Safe action dictionary or None if it can't be made safe
        """
        if action_name not in self.ros_action_schemas:
            logger.error(f"Unknown action: {action_name}")
            return None

        # Sanitize coordinates first
        sanitized_args = self.sanitize_coordinates(action_args)

        # Apply position limits
        if action_name in ["move_to_position", "grasp_object"]:
            for coord in ["x", "y", "z"]:
                if coord in sanitized_args:
                    limits = self.safety_constraints["position_limits"][coord]
                    value = sanitized_args[coord]
                    # Clamp the value to safe limits
                    sanitized_args[coord] = max(limits["min"], min(limits["max"], value))

        # Validate the safe version
        validation_result = self.validate_action_json(action_name, sanitized_args)

        if validation_result["valid"]:
            return {
                "function": action_name,
                "arguments": sanitized_args,
                "was_sanitized": sanitized_args != action_args
            }
        else:
            logger.warning(f"Could not create safe action: {validation_result['errors']}")
            return None


def test_json_validator():
    """Test function for JSON validator functionality."""
    print("Testing JSON validator...")

    # Create validator instance
    validator = JSONValidator()

    # Test 1: Valid move_to_position action
    valid_move = {
        "function": "move_to_position",
        "arguments": {"x": 1.5, "y": 2.0, "z": 0.0}
    }
    result1 = validator.validate_action_json(valid_move["function"], valid_move["arguments"])
    print(f"Valid move validation: {result1['valid']}")

    # Test 2: Invalid move_to_position action (out of bounds)
    invalid_move = {
        "function": "move_to_position",
        "arguments": {"x": 15.0, "y": 2.0}  # x is out of bounds
    }
    result2 = validator.validate_action_json(invalid_move["function"], invalid_move["arguments"])
    print(f"Invalid move validation: {result2['valid']}, errors: {result2['errors']}")

    # Test 3: Valid action sequence
    action_sequence = [
        {"function": "move_to_position", "arguments": {"x": 1.0, "y": 1.0}},
        {"function": "grasp_object", "arguments": {"object_description": "red apple"}}
    ]
    result3 = validator.validate_action_sequence(action_sequence)
    print(f"Action sequence validation: {result3['valid']}, errors: {result3['errors']}")

    # Test 4: JSON string validation
    json_str = '{"function": "navigate_to", "arguments": {"location": "kitchen"}}'
    result4 = validator.validate_json_string(json_str)
    print(f"JSON string validation: {result4['valid']}")

    # Test 5: Safe action creation
    unsafe_action = {
        "function": "move_to_position",
        "arguments": {"x": 15.0, "y": 20.0, "z": 3.0}  # Out of bounds
    }
    safe_action = validator.create_safe_action(unsafe_action["function"], unsafe_action["arguments"])
    print(f"Safe action creation: {safe_action}")

    # Test 6: Coordinate sanitization
    unsanitized_args = {"x": 1.234567, "y": 2.345678, "z": 0.456789}
    sanitized_args = validator.sanitize_coordinates(unsanitized_args)
    print(f"Coordinate sanitization: {unsanitized_args} -> {sanitized_args}")

    # Check if all tests passed appropriately
    success = (
        result1["valid"] and  # Valid move should pass
        not result2["valid"] and  # Invalid move should fail
        result3["valid"] and  # Valid sequence should pass
        result4["valid"] and  # Valid JSON should pass
        safe_action is not None and  # Safe action should be created
        all(abs(sanitized_args[k] - round(unsanitized_args[k], 3)) < 1e-10 for k in unsanitized_args.keys())  # Sanitization correct
    )

    if success:
        print("✓ JSON validator tests passed")
        return True
    else:
        print("✗ JSON validator tests failed")
        return False


if __name__ == "__main__":
    test_json_validator()