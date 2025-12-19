"""
Safety Guardrails Module for VLA System

This module implements safety validation to prevent dangerous robot actions
based on semantic analysis, action validation, and environmental context.
"""
import re
from typing import Dict, Any, List, Optional, Union
import logging
from enum import Enum

logger = logging.getLogger(__name__)

class RiskLevel(Enum):
    """Enumeration for risk levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

class SafetyGuardrails:
    """
    Class for implementing safety validation and guardrails for robot actions.
    """
    def __init__(self, risk_threshold: float = 0.8, confirmation_required_risk: float = 0.6):
        """
        Initialize the safety guardrails system.

        Args:
            risk_threshold: Threshold above which actions are blocked (0.0-1.0)
            confirmation_required_risk: Threshold above which human confirmation is required (0.0-1.0)
        """
        self.risk_threshold = risk_threshold
        self.confirmation_required_risk = confirmation_required_risk

        # Define dangerous keywords and patterns
        self.dangerous_keywords = {
            'physical_harm': [
                'harm', 'injure', 'hurt', 'damage', 'break', 'destroy', 'crash', 'hit', 'crush',
                'crack', 'smash', 'shatter', 'wreck', 'demolish', 'ruin', 'ruin', 'vandalize'
            ],
            'human_safety': [
                'human', 'person', 'people', 'body', 'head', 'arm', 'leg', 'face', 'hand',
                'child', 'elderly', 'infant', 'pet', 'animal'
            ],
            'environmental_hazard': [
                'fire', 'water', 'heat', 'hot', 'cold', 'freeze', 'electric', 'shock', 'spark',
                'chemical', 'acid', 'toxic', 'poison', 'radiation', 'gas', 'explosive'
            ],
            'forbidden_actions': [
                'attack', 'fight', 'aggressive', 'threaten', 'scare', 'frighten', 'intimidate',
                'chase', 'pursue', 'trap', 'capture', 'imprison', 'confine'
            ]
        }

        # Define safe action patterns
        self.safe_actions = [
            'move_to', 'navigate_to', 'grasp', 'pick_up', 'place', 'put_down', 'lift',
            'lower', 'rotate', 'turn', 'stop', 'wait', 'pause', 'detect', 'find', 'identify'
        ]

        # Define dangerous action patterns
        self.dangerous_actions = [
            'crush', 'smash', 'destroy', 'break', 'hit', 'attack', 'harm', 'injure',
            'damage', 'vandalize', 'demolish'
        ]

        # Position safety limits
        self.position_limits = {
            'x': {'min': -5.0, 'max': 5.0},
            'y': {'min': -5.0, 'max': 5.0},
            'z': {'min': 0.0, 'max': 2.0}  # Z should be above ground
        }

        # Speed and force limits
        self.speed_limits = {
            'linear': 1.0,  # m/s
            'angular': 0.5  # rad/s
        }

    def validate_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate a single robot action against safety requirements.

        Args:
            action: Action dictionary with function and arguments

        Returns:
            Dictionary with validation results
        """
        if not isinstance(action, dict) or 'function' not in action:
            return {
                "valid": False,
                "risk_level": RiskLevel.CRITICAL,
                "confidence": 1.0,
                "violations": ["Action is not properly formatted"],
                "requires_confirmation": False
            }

        action_function = action.get('function', '').lower()
        action_args = action.get('arguments', {})

        # Check for dangerous action patterns
        violations = []
        risk_score = 0.0

        # Check if action is in forbidden list
        if action_function in self.dangerous_actions:
            violations.append(f"Action '{action_function}' is in forbidden actions list")
            risk_score = 1.0  # Maximum risk

        # Check arguments for dangerous content
        dangerous_arg_violations = self._check_dangerous_arguments(action_args)
        violations.extend(dangerous_arg_violations)
        if dangerous_arg_violations:
            risk_score = max(risk_score, 0.9)

        # Check position limits
        position_violations = self._check_position_limits(action_args)
        violations.extend(position_violations)
        if position_violations:
            risk_score = max(risk_score, 0.7)

        # Determine risk level
        if risk_score >= self.risk_threshold:
            risk_level = RiskLevel.CRITICAL
        elif risk_score >= self.confirmation_required_risk:
            risk_level = RiskLevel.HIGH
        elif risk_score > 0.3:
            risk_level = RiskLevel.MEDIUM
        else:
            risk_level = RiskLevel.LOW

        # Determine if confirmation is required
        requires_confirmation = risk_level.value in [RiskLevel.HIGH.value, RiskLevel.CRITICAL.value]

        return {
            "valid": risk_level.value in [RiskLevel.LOW.value, RiskLevel.MEDIUM.value],
            "risk_level": risk_level,
            "confidence": 1.0 - risk_score,  # Higher confidence for lower risk
            "violations": violations,
            "requires_confirmation": requires_confirmation,
            "action_function": action_function,
            "risk_score": risk_score
        }

    def _check_dangerous_arguments(self, args: Dict[str, Any]) -> List[str]:
        """
        Check action arguments for dangerous content.

        Args:
            args: Action arguments dictionary

        Returns:
            List of violations found
        """
        violations = []

        for key, value in args.items():
            if isinstance(value, str):
                # Check for dangerous keywords in string arguments
                value_lower = value.lower()

                for category, keywords in self.dangerous_keywords.items():
                    for keyword in keywords:
                        if keyword in value_lower:
                            violations.append(f"Found dangerous keyword '{keyword}' in {category} category in argument '{key}'")

        return violations

    def _check_position_limits(self, args: Dict[str, Any]) -> List[str]:
        """
        Check if position arguments are within safe limits.

        Args:
            args: Action arguments dictionary

        Returns:
            List of position limit violations
        """
        violations = []

        # Check for position-related arguments
        for coord in ['x', 'y', 'z']:
            if coord in args:
                value = args[coord]
                if isinstance(value, (int, float)):
                    limits = self.position_limits[coord]
                    if not (limits['min'] <= value <= limits['max']):
                        violations.append(
                            f"Coordinate {coord}={value} is outside safe limits [{limits['min']}, {limits['max']}]"
                        )

        return violations

    def validate_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate a sequence of actions for safety.

        Args:
            action_sequence: List of action dictionaries

        Returns:
            Dictionary with validation results for the entire sequence
        """
        total_risk = 0.0
        all_violations = []
        individual_results = []
        valid_count = 0
        requires_confirmation_count = 0

        for i, action in enumerate(action_sequence):
            result = self.validate_action(action)
            individual_results.append(result)

            if result["valid"]:
                valid_count += 1

            if result["requires_confirmation"]:
                requires_confirmation_count += 1

            # Add to total violations
            all_violations.extend([f"Action {i}: {v}" for v in result["violations"]])

            # Update total risk (use the highest risk in the sequence)
            total_risk = max(total_risk, 1.0 - result["confidence"])

        # Overall sequence is valid only if all actions are valid
        sequence_valid = all(result["valid"] for result in individual_results)

        return {
            "valid": sequence_valid,
            "total_risk": total_risk,
            "violations": all_violations,
            "individual_results": individual_results,
            "total_actions": len(action_sequence),
            "valid_actions": valid_count,
            "requires_confirmation_count": requires_confirmation_count,
            "sequence_risk_level": self._risk_score_to_level(total_risk)
        }

    def _risk_score_to_level(self, risk_score: float) -> RiskLevel:
        """
        Convert a risk score to a risk level.

        Args:
            risk_score: Risk score between 0.0 and 1.0

        Returns:
            Corresponding RiskLevel
        """
        if risk_score >= self.risk_threshold:
            return RiskLevel.CRITICAL
        elif risk_score >= self.confirmation_required_risk:
            return RiskLevel.HIGH
        elif risk_score > 0.3:
            return RiskLevel.MEDIUM
        else:
            return RiskLevel.LOW

    def check_environmental_safety(self, action: Dict[str, Any], environment_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Check if an action is safe given the environmental context.

        Args:
            action: Action to validate
            environment_context: Environmental information (objects, humans, obstacles)

        Returns:
            Dictionary with environmental safety validation results
        """
        if not environment_context:
            return {
                "safe": True,
                "confidence": 0.5,  # Low confidence without context
                "reasons": ["No environmental context provided"]
            }

        action_function = action.get('function', '').lower()
        action_args = action.get('arguments', {})

        safety_issues = []
        confidence = 1.0

        # Check for humans in the environment
        humans = environment_context.get('humans', [])
        if humans and action_function in ['navigate_to', 'move_to_position', 'grasp']:
            # Check if action path intersects with human positions
            for human in humans:
                if self._is_path_conflict(action_args, human.get('position', {})):
                    safety_issues.append(f"Action path conflicts with human at {human.get('position', {})}")
                    confidence = 0.1

        # Check for fragile objects
        fragile_objects = environment_context.get('fragile_objects', [])
        if fragile_objects and action_function in ['move_to_position', 'navigate_to', 'grasp']:
            for obj in fragile_objects:
                if self._is_too_close(action_args, obj.get('position', {})):
                    safety_issues.append(f"Action too close to fragile object: {obj.get('class', 'unknown')}")
                    confidence *= 0.5  # Reduce confidence

        # Check for obstacles
        obstacles = environment_context.get('obstacles', [])
        if obstacles and action_function in ['navigate_to', 'move_to_position']:
            for obstacle in obstacles:
                if self._is_path_conflict(action_args, obstacle.get('position', {})):
                    safety_issues.append(f"Action path blocked by obstacle")
                    confidence *= 0.3

        return {
            "safe": len(safety_issues) == 0,
            "confidence": confidence,
            "reasons": safety_issues,
            "environment_considered": True
        }

    def _is_path_conflict(self, action_args: Dict[str, Any], object_position: Dict[str, Any]) -> bool:
        """
        Check if an action path conflicts with an object position.

        Args:
            action_args: Arguments for the action
            object_position: Position of the object to check against

        Returns:
            True if there's a conflict, False otherwise
        """
        # This is a simplified check - in reality, path planning would be more sophisticated
        if 'x' in action_args and 'y' in action_args and object_position:
            action_x = action_args.get('x', 0)
            action_y = action_args.get('y', 0)
            obj_x = object_position.get('x', 0)
            obj_y = object_position.get('y', 0)

            # Check if the action destination is too close to the object
            distance = ((action_x - obj_x)**2 + (action_y - obj_y)**2)**0.5
            return distance < 0.5  # 50cm threshold

        return False

    def _is_too_close(self, action_args: Dict[str, Any], object_position: Dict[str, Any]) -> bool:
        """
        Check if an action is too close to an object.

        Args:
            action_args: Arguments for the action
            object_position: Position of the object to check against

        Returns:
            True if too close, False otherwise
        """
        if 'x' in action_args and 'y' in action_args and object_position:
            action_x = action_args.get('x', 0)
            action_y = action_args.get('y', 0)
            obj_x = object_position.get('x', 0)
            obj_y = object_position.get('y', 0)

            distance = ((action_x - obj_x)**2 + (action_y - obj_y)**2)**0.5
            return distance < 0.3  # 30cm threshold for fragile objects

        return False

    def validate_with_context(self, action: Dict[str, Any], environment_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Validate an action with both intrinsic safety checks and environmental context.

        Args:
            action: Action to validate
            environment_context: Environmental context for safety check

        Returns:
            Combined validation results
        """
        # First, run intrinsic safety validation
        intrinsic_result = self.validate_action(action)

        # Then, run environmental safety validation
        environmental_result = self.check_environmental_safety(action, environment_context)

        # Combine results
        combined_risk = max(
            1.0 - intrinsic_result['confidence'],
            1.0 - environmental_result['confidence'] if environmental_result['environment_considered'] else 0.0
        )

        combined_risk_level = self._risk_score_to_level(combined_risk)

        return {
            "intrinsic_validation": intrinsic_result,
            "environmental_validation": environmental_result,
            "combined_risk_level": combined_risk_level,
            "combined_risk_score": combined_risk,
            "overall_valid": intrinsic_result['valid'] and environmental_result['safe'],
            "requires_confirmation": intrinsic_result['requires_confirmation'] or not environmental_result['safe']
        }

    def get_safety_report(self, action_sequence: List[Dict[str, Any]], environment_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Generate a comprehensive safety report for an action sequence.

        Args:
            action_sequence: Sequence of actions to analyze
            environment_context: Environmental context

        Returns:
            Comprehensive safety report
        """
        sequence_validation = self.validate_action_sequence(action_sequence)

        individual_reports = []
        for action in action_sequence:
            report = self.validate_with_context(action, environment_context)
            individual_reports.append(report)

        # Calculate overall metrics
        total_risk_score = max(report['combined_risk_score'] for report in individual_reports) if individual_reports else 0.0
        avg_risk_score = sum(report['combined_risk_score'] for report in individual_reports) / len(individual_reports) if individual_reports else 0.0

        high_risk_actions = [i for i, report in enumerate(individual_reports)
                            if report['combined_risk_level'] in [RiskLevel.HIGH, RiskLevel.CRITICAL]]

        return {
            "sequence_validation": sequence_validation,
            "individual_reports": individual_reports,
            "total_risk_score": total_risk_score,
            "average_risk_score": avg_risk_score,
            "high_risk_action_indices": high_risk_actions,
            "total_actions": len(action_sequence),
            "environment_context_considered": environment_context is not None,
            "safety_summary": {
                "total_risk_level": self._risk_score_to_level(total_risk_score),
                "requires_human_confirmation": len(high_risk_actions) > 0,
                "safe_action_count": len(action_sequence) - len(high_risk_actions),
                "unsafe_action_count": len(high_risk_actions)
            }
        }


def test_safety_guardrails():
    """Test function for safety guardrails functionality."""
    print("Testing safety guardrails...")

    # Create safety guardrails instance
    guardrails = SafetyGuardrails(risk_threshold=0.6, confirmation_required_risk=0.3)

    # Test 1: Safe action
    safe_action = {
        "function": "move_to_position",
        "arguments": {"x": 1.0, "y": 1.0, "z": 0.5}
    }
    result1 = guardrails.validate_action(safe_action)
    print(f"Safe action validation: valid={result1['valid']}, risk={result1['risk_level'].value}")

    # Test 2: Dangerous action
    dangerous_action = {
        "function": "harm",
        "arguments": {"target": "human"}
    }
    result2 = guardrails.validate_action(dangerous_action)
    print(f"Dangerous action validation: valid={result2['valid']}, risk={result2['risk_level'].value}")

    # Test 3: Action with dangerous argument
    risky_action = {
        "function": "move_to_position",
        "arguments": {"x": 10.0, "y": 1.0}  # x is outside safe limits
    }
    result3 = guardrails.validate_action(risky_action)
    print(f"Position violation: valid={result3['valid']}, violations={result3['violations']}")

    # Test 4: Action sequence
    action_sequence = [safe_action, risky_action]
    sequence_result = guardrails.validate_action_sequence(action_sequence)
    print(f"Action sequence: valid={sequence_result['valid']}, total_risk={sequence_result['total_risk']:.2f}")

    # Test 5: Environmental safety check
    environment_context = {
        "humans": [{"position": {"x": 1.1, "y": 1.1, "z": 0.0}}],
        "fragile_objects": [{"class": "vase", "position": {"x": 0.2, "y": 0.2, "z": 0.5}}],
        "obstacles": []
    }
    env_result = guardrails.check_environmental_safety(safe_action, environment_context)
    print(f"Environmental check: safe={env_result['safe']}, reasons={env_result['reasons']}")

    # Test 6: Combined validation
    combined_result = guardrails.validate_with_context(safe_action, environment_context)
    print(f"Combined validation: overall_valid={combined_result['overall_valid']}")

    # Test 7: Safety report
    safety_report = guardrails.get_safety_report([safe_action, risky_action], environment_context)
    print(f"Safety report: {safety_report['safety_summary']}")

    # Check if tests passed appropriately
    success = (
        result1['valid'] and result1['risk_level'] == RiskLevel.LOW and  # Safe action should pass
        not result2['valid'] and result2['risk_level'] == RiskLevel.CRITICAL and  # Dangerous action should fail
        len(result3['violations']) > 0 and not result3['valid'] and  # Position violation should be detected
        not sequence_result['valid'] and sequence_result['total_risk'] > 0.5 and  # Sequence with risky action should fail
        combined_result['overall_valid'] == False  # Combined check with environment should detect human conflict
    )

    if success:
        print("✓ Safety guardrails tests passed")
        return True
    else:
        print("✗ Safety guardrails tests failed")
        return False


if __name__ == "__main__":
    test_safety_guardrails()