"""
Vision-Language Grounding Pipeline for VLA System

This module connects linguistic descriptions to visual objects,
enabling the system to understand which specific object the user is referring to.
"""
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import logging
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import re

logger = logging.getLogger(__name__)

class GroundingPipeline:
    """
    Class for connecting linguistic descriptions to visual objects.
    """
    def __init__(self, detection_threshold: float = 0.7, grounding_threshold: float = 0.5):
        """
        Initialize the grounding pipeline.

        Args:
            detection_threshold: Minimum confidence for object detection
            grounding_threshold: Minimum similarity score for grounding
        """
        self.detection_threshold = detection_threshold
        self.grounding_threshold = grounding_threshold

        # Initialize TF-IDF vectorizer for text similarity
        self.tfidf_vectorizer = TfidfVectorizer(
            lowercase=True,
            stop_words='english',
            ngram_range=(1, 2)  # Use unigrams and bigrams
        )

        # Object class synonyms for better matching
        self.class_synonyms = {
            'bottle': ['container', 'drink', 'liquid', 'vessel'],
            'apple': ['fruit', 'red fruit', 'snack'],
            'box': ['container', 'package', 'carton', 'crate'],
            'cup': ['glass', 'mug', 'container', 'drink'],
            'chair': ['seat', 'furniture'],
            'table': ['furniture', 'surface'],
            'person': ['human', 'man', 'woman', 'person'],
            'dog': ['pet', 'animal', 'canine'],
            'cat': ['pet', 'animal', 'feline']
        }

    def ground_language_to_objects(self,
                                 language_description: str,
                                 detected_objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Ground a linguistic description to specific visual objects.

        Args:
            language_description: Natural language description of the target object
            detected_objects: List of detected objects from vision system

        Returns:
            The best matching object, or None if no good match found
        """
        if not detected_objects:
            logger.warning("No detected objects to ground language to")
            return None

        # Preprocess the language description
        processed_description = self._preprocess_text(language_description)

        # Calculate similarity between description and each object
        best_match = None
        best_score = -1.0

        for obj in detected_objects:
            # Calculate grounding score
            score = self._calculate_grounding_score(processed_description, obj)

            logger.debug(f"Grounding score for {obj['class']}: {score:.3f}")

            if score > best_score and score >= self.grounding_threshold:
                best_score = score
                best_match = obj

        if best_match:
            # Add grounding confidence to the result
            result = best_match.copy()
            result['grounding_confidence'] = best_score
            result['language_description'] = language_description
            logger.info(f"Grounded '{language_description}' to {best_match['class']} with confidence {best_score:.3f}")
            return result
        else:
            logger.info(f"No object found matching description: '{language_description}'")
            return None

    def _preprocess_text(self, text: str) -> str:
        """
        Preprocess text for better matching.

        Args:
            text: Input text to preprocess

        Returns:
            Preprocessed text
        """
        # Convert to lowercase
        text = text.lower()

        # Remove special characters but keep spaces and basic punctuation
        text = re.sub(r'[^\w\s]', ' ', text)

        # Remove extra whitespace
        text = ' '.join(text.split())

        return text

    def _calculate_grounding_score(self, language_description: str, obj: Dict[str, Any]) -> float:
        """
        Calculate the grounding score between a language description and an object.

        Args:
            language_description: Preprocessed language description
            obj: Detected object dictionary

        Returns:
            Grounding score (0.0 to 1.0)
        """
        # Extract object information
        obj_class = obj.get('class', '').lower()
        obj_color = obj.get('color', '').lower()
        obj_id = obj.get('id', '')

        # Create object description for comparison
        obj_descriptions = [obj_class]

        # Add color if available
        if obj_color:
            obj_descriptions.append(f"{obj_color} {obj_class}")

        # Add synonyms
        if obj_class in self.class_synonyms:
            obj_descriptions.extend(self.class_synonyms[obj_class])

        # Calculate similarity between language description and each object description
        max_similarity = 0.0
        for obj_desc in obj_descriptions:
            similarity = self._text_similarity(language_description, obj_desc)
            max_similarity = max(max_similarity, similarity)

        # Also consider position information if available
        position_score = self._position_score(language_description, obj)

        # Combine similarity and position scores
        combined_score = 0.8 * max_similarity + 0.2 * position_score

        return combined_score

    def _text_similarity(self, text1: str, text2: str) -> float:
        """
        Calculate text similarity using TF-IDF and cosine similarity.

        Args:
            text1: First text
            text2: Second text

        Returns:
            Similarity score (0.0 to 1.0)
        """
        try:
            # Fit and transform the texts
            tfidf_matrix = self.tfidf_vectorizer.fit_transform([text1, text2])

            # Calculate cosine similarity
            similarity_matrix = cosine_similarity(tfidf_matrix[0:1], tfidf_matrix[1:2])
            similarity = similarity_matrix[0][0]

            return float(similarity)
        except:
            # Fallback to simple word overlap
            words1 = set(text1.split())
            words2 = set(text2.split())
            intersection = words1.intersection(words2)
            union = words1.union(words2)

            if len(union) == 0:
                return 0.0

            return len(intersection) / len(union)

    def _position_score(self, language_description: str, obj: Dict[str, Any]) -> float:
        """
        Calculate score based on positional language (e.g., "left", "right", "front").

        Args:
            language_description: Language description
            obj: Object with position information

        Returns:
            Position-based score (0.0 to 1.0)
        """
        description_lower = language_description.lower()

        # Extract position information from object
        obj_position = obj.get('position_3d', {})
        obj_x = obj_position.get('x', 0.0)
        obj_y = obj_position.get('y', 0.0)

        # Check for positional words in description
        position_words = {
            'left': (-1, 0),  # Negative x direction
            'right': (1, 0),  # Positive x direction
            'front': (0, 1),  # Positive y direction
            'back': (0, -1),  # Negative y direction
            'behind': (0, -1),
            'in front of': (0, 1),
            'near': (0, 0),  # No specific direction
            'close': (0, 0)
        }

        # Find position-relevant words in description
        max_position_score = 0.0
        for word, direction in position_words.items():
            if word in description_lower:
                # Calculate how well the object position matches the expected direction
                # For simplicity, we'll just check if the object is in the right quadrant
                expected_x, expected_y = direction

                if expected_x == 0 and expected_y == 0:
                    # "Near" or "close" - any object gets some score
                    position_score = 0.5
                elif expected_x != 0:
                    # Check x direction
                    position_score = 0.7 if (obj_x * expected_x > 0) else 0.1
                elif expected_y != 0:
                    # Check y direction
                    position_score = 0.7 if (obj_y * expected_y > 0) else 0.1
                else:
                    position_score = 0.0

                max_position_score = max(max_position_score, position_score)

        return max_position_score

    def disambiguate_objects(self,
                           language_description: str,
                           candidate_objects: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Disambiguate between multiple similar objects based on language description.

        Args:
            language_description: Language description that should disambiguate
            candidate_objects: List of similar objects to disambiguate between

        Returns:
            List of objects with grounding scores, sorted by score
        """
        if not candidate_objects:
            return []

        processed_description = self._preprocess_text(language_description)

        # Calculate grounding scores for all candidates
        scored_objects = []
        for obj in candidate_objects:
            score = self._calculate_grounding_score(processed_description, obj)
            scored_obj = obj.copy()
            scored_obj['grounding_score'] = score
            scored_objects.append(scored_obj)

        # Sort by grounding score (highest first)
        scored_objects.sort(key=lambda x: x['grounding_score'], reverse=True)

        # Filter by grounding threshold
        filtered_objects = [obj for obj in scored_objects if obj['grounding_score'] >= self.grounding_threshold]

        return filtered_objects

    def disambiguate_by_relative_position(self,
                                        language_description: str,
                                        candidate_objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Disambiguate objects based on relative position language (e.g., "the left one", "the one on the right").

        Args:
            language_description: Language description with position information
            candidate_objects: List of similar objects to disambiguate between

        Returns:
            The object that matches the position description, or None if no clear match
        """
        if not candidate_objects:
            return None

        processed_description = self._preprocess_text(language_description)

        # Extract position information from description
        position_indicators = {
            'left': [-1, 0],      # Negative x direction
            'right': [1, 0],      # Positive x direction
            'front': [0, 1],      # Positive y direction (assuming robot perspective)
            'back': [0, -1],      # Negative y direction
            'behind': [0, -1],
            'in front': [0, 1],
            'above': [0, 0, 1],   # Positive z direction
            'below': [0, 0, -1],  # Negative z direction
            'top': [0, 0, 1],
            'bottom': [0, 0, -1],
            'center': [0, 0],     # Central position
            'middle': [0, 0]
        }

        # Find which position indicators are in the description
        found_positions = []
        for word, direction in position_indicators.items():
            if word in processed_description:
                found_positions.append((word, direction))

        if not found_positions:
            # No position information found, return the highest-scoring object
            if candidate_objects:
                processed_desc = self._preprocess_text(language_description)
                scored_objects = []
                for obj in candidate_objects:
                    score = self._calculate_grounding_score(processed_desc, obj)
                    scored_objects.append((obj, score))

                scored_objects.sort(key=lambda x: x[1], reverse=True)
                return scored_objects[0][0]  # Return the best match based on description alone
            return None

        # Get the first position indicator found (priority to earlier ones)
        position_word, direction = found_positions[0]

        # Sort objects by their position relative to the requested direction
        if len(direction) == 2:  # 2D direction (x, y)
            dx, dy = direction
            if dx != 0:  # Left/right
                candidate_objects.sort(key=lambda obj: obj.get('position_3d', {}).get('x', 0) * dx, reverse=(dx > 0))
            elif dy != 0:  # Front/back
                candidate_objects.sort(key=lambda obj: obj.get('position_3d', {}).get('y', 0) * dy, reverse=(dy > 0))
            else:  # Center/middle
                # Find object closest to center (x=0, y=0)
                candidate_objects.sort(key=lambda obj: abs(obj.get('position_3d', {}).get('x', 0)) + abs(obj.get('position_3d', {}).get('y', 0)))
        elif len(direction) == 3:  # 3D direction (x, y, z)
            dx, dy, dz = direction
            if dz != 0:  # Up/down
                candidate_objects.sort(key=lambda obj: obj.get('position_3d', {}).get('z', 0) * dz, reverse=(dz > 0))

        # Return the most relevant object based on position
        return candidate_objects[0] if candidate_objects else None

    def disambiguate_by_attribute(self,
                                  language_description: str,
                                  candidate_objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Disambiguate objects based on specific attributes mentioned in the description.

        Args:
            language_description: Language description with attribute information
            candidate_objects: List of similar objects to disambiguate between

        Returns:
            The object that best matches the attributes, or None if no clear match
        """
        if not candidate_objects:
            return None

        processed_description = self._preprocess_text(language_description)

        # Look for size descriptors
        size_indicators = {
            'large': lambda obj: _get_size_score(obj, 'large'),
            'big': lambda obj: _get_size_score(obj, 'large'),
            'huge': lambda obj: _get_size_score(obj, 'large'),
            'small': lambda obj: _get_size_score(obj, 'small'),
            'tiny': lambda obj: _get_size_score(obj, 'small'),
            'little': lambda obj: _get_size_score(obj, 'small'),
            'medium': lambda obj: _get_size_score(obj, 'medium'),
            'average': lambda obj: _get_size_score(obj, 'medium')
        }

        # Look for color descriptors
        color_indicators = {
            'red': 'red',
            'blue': 'blue',
            'green': 'green',
            'yellow': 'yellow',
            'orange': 'orange',
            'purple': 'purple',
            'pink': 'pink',
            'black': 'black',
            'white': 'white',
            'gray': 'gray',
            'brown': 'brown'
        }

        # Find matches based on attributes
        potential_matches = []

        for obj in candidate_objects:
            score = 0

            # Check for size matches
            for size_word, size_func in size_indicators.items():
                if size_word in processed_description:
                    score += size_func(obj)

            # Check for color matches
            for color_word, color_val in color_indicators.items():
                if color_word in processed_description:
                    # In a real system, we would check the actual color from the image
                    # For now, we'll just see if the color is mentioned in the object's class
                    if color_val in obj.get('class', '').lower():
                        score += 2  # Strong match for color

            # Check for class matches
            obj_class = obj.get('class', '').lower()
            if obj_class in processed_description:
                score += 3  # Strong match for class

            potential_matches.append((obj, score))

        # Sort by score and return the best match if it meets threshold
        potential_matches.sort(key=lambda x: x[1], reverse=True)

        best_match, best_score = potential_matches[0] if potential_matches else (None, 0)

        # Only return if we have a reasonably confident match
        if best_score > 0:
            return best_match
        else:
            # Fall back to description-based grounding
            processed_desc = self._preprocess_text(language_description)
            scored_objects = []
            for obj in candidate_objects:
                score = self._calculate_grounding_score(processed_desc, obj)
                scored_objects.append((obj, score))

            scored_objects.sort(key=lambda x: x[1], reverse=True)
            return scored_objects[0][0] if scored_objects else None

    def comprehensive_disambiguation(self,
                                   language_description: str,
                                   candidate_objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Perform comprehensive disambiguation using multiple strategies.

        Args:
            language_description: Full language description for disambiguation
            candidate_objects: List of similar objects to disambiguate between

        Returns:
            The most likely object based on comprehensive analysis, or None if no clear match
        """
        if not candidate_objects:
            return None

        # First, try position-based disambiguation (often most reliable)
        position_match = self.disambiguate_by_relative_position(language_description, candidate_objects)

        if position_match:
            # Verify that the position-based match is semantically reasonable
            processed_desc = self._preprocess_text(language_description)
            position_score = self._calculate_grounding_score(processed_desc, position_match)

            if position_score >= self.grounding_threshold:
                return position_match

        # Next, try attribute-based disambiguation
        attribute_match = self.disambiguate_by_attribute(language_description, candidate_objects)

        if attribute_match:
            # Verify that the attribute-based match is semantically reasonable
            processed_desc = self._preprocess_text(language_description)
            attribute_score = self._calculate_grounding_score(processed_desc, attribute_match)

            if attribute_score >= self.grounding_threshold:
                return attribute_match

        # Finally, fall back to general disambiguation
        general_matches = self.disambiguate_objects(language_description, candidate_objects)

        if general_matches:
            return general_matches[0]  # Return highest scoring match

        # If all strategies fail, return the best general match
        processed_desc = self._preprocess_text(language_description)
        all_scored = [(obj, self._calculate_grounding_score(processed_desc, obj)) for obj in candidate_objects]
        all_scored.sort(key=lambda x: x[1], reverse=True)

        best_obj, best_score = all_scored[0] if all_scored else (None, 0)
        return best_obj if best_score >= 0.1 else None  # Very low threshold fallback


def _get_size_score(obj: Dict[str, Any], target_size: str) -> int:
    """
    Helper function to score an object based on its size relative to a target size.

    Args:
        obj: Object to score
        target_size: Target size ("large", "small", "medium")

    Returns:
        Score based on size match (0-2)
    """
    # Extract size from bounding box
    bbox = obj.get('bbox', {})
    if not bbox:
        return 0

    width = bbox.get('x_max', 0) - bbox.get('x_min', 0)
    height = bbox.get('y_max', 0) - bbox.get('y_min', 0)
    area = width * height

    # Define size thresholds (these would be calibrated in a real system)
    if target_size == 'large':
        return 2 if area > 8000 else (1 if area > 4000 else 0)
    elif target_size == 'small':
        return 2 if area < 3000 else (1 if area < 6000 else 0)
    elif target_size == 'medium':
        return 2 if 3000 <= area <= 7000 else (1 if 2000 <= area <= 9000 else 0)
    else:
        return 0

    def ground_to_multiple_objects(self,
                                 language_description: str,
                                 detected_objects: List[Dict[str, Any]],
                                 max_objects: int = 3) -> List[Dict[str, Any]]:
        """
        Ground a language description to multiple objects (e.g., "all red objects").

        Args:
            language_description: Language description
            detected_objects: List of detected objects
            max_objects: Maximum number of objects to return

        Returns:
            List of objects that match the description
        """
        if not detected_objects:
            return []

        processed_description = self._preprocess_text(language_description)
        matching_objects = []

        for obj in detected_objects:
            score = self._calculate_grounding_score(processed_description, obj)
            if score >= self.grounding_threshold:
                result = obj.copy()
                result['grounding_confidence'] = score
                matching_objects.append(result)

        # Sort by confidence score
        matching_objects.sort(key=lambda x: x['grounding_confidence'], reverse=True)

        # Limit to max_objects
        return matching_objects[:max_objects]

    def create_grounding_context(self,
                               language_description: str,
                               detected_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Create a grounding context that combines vision and language information.

        Args:
            language_description: User's language description
            detected_objects: Objects detected in the scene

        Returns:
            Dictionary with grounding context information
        """
        # Find the best matching object
        best_match = self.ground_language_to_objects(language_description, detected_objects)

        # Find all matching objects
        all_matches = self.ground_to_multiple_objects(language_description, detected_objects)

        # Create grounding context
        grounding_context = {
            "language_description": language_description,
            "detected_objects_count": len(detected_objects),
            "best_match": best_match,
            "all_matches": all_matches,
            "grounding_successful": best_match is not None,
            "objects_with_scores": []
        }

        # Add scores for all objects
        processed_description = self._preprocess_text(language_description)
        for obj in detected_objects:
            score = self._calculate_grounding_score(processed_description, obj)
            grounding_context["objects_with_scores"].append({
                "id": obj.get("id"),
                "class": obj.get("class"),
                "confidence": obj.get("confidence"),
                "grounding_score": score
            })

        # Sort objects by grounding score
        grounding_context["objects_with_scores"].sort(key=lambda x: x["grounding_score"], reverse=True)

        return grounding_context

    def validate_grounding(self,
                         language_description: str,
                         grounded_object: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate the quality of the grounding result.

        Args:
            language_description: Original language description
            grounded_object: Object that was grounded

        Returns:
            Dictionary with validation results
        """
        if grounded_object is None:
            return {
                "valid": False,
                "confidence": 0.0,
                "reason": "No object grounded"
            }

        grounding_confidence = grounded_object.get('grounding_confidence', 0.0)

        # Check if the grounding confidence is above threshold
        is_confident = grounding_confidence >= self.grounding_threshold

        # Check if the object class makes sense with the description
        obj_class = grounded_object.get('class', '').lower()
        desc_lower = language_description.lower()

        # Simple semantic check
        semantic_match = any(word in desc_lower for word in [obj_class] + self.class_synonyms.get(obj_class, []))

        validation_result = {
            "valid": is_confident and semantic_match,
            "confidence": grounding_confidence,
            "semantic_match": semantic_match,
            "is_confident": is_confident
        }

        if not validation_result["valid"]:
            reasons = []
            if not is_confident:
                reasons.append(f"Grounding confidence {grounding_confidence:.3f} below threshold {self.grounding_threshold}")
            if not semantic_match:
                reasons.append(f"Object class '{obj_class}' doesn't semantically match description")
            validation_result["reason"] = "; ".join(reasons)

        return validation_result


def test_grounding_pipeline():
    """Test function for grounding pipeline functionality."""
    print("Testing grounding pipeline...")

    # Create grounding pipeline instance
    pipeline = GroundingPipeline(detection_threshold=0.5, grounding_threshold=0.3)

    # Create mock detected objects
    mock_objects = [
        {
            "id": "obj_001",
            "class": "bottle",
            "confidence": 0.89,
            "bbox": {"x_min": 100, "y_min": 150, "x_max": 180, "y_max": 230},
            "position_3d": {"x": -0.5, "y": 0.8, "z": 0.8},
            "color": "blue"
        },
        {
            "id": "obj_002",
            "class": "apple",
            "confidence": 0.92,
            "bbox": {"x_min": 250, "y_min": 120, "x_max": 320, "y_max": 200},
            "position_3d": {"x": 0.5, "y": 0.8, "z": 0.8},
            "color": "red"
        },
        {
            "id": "obj_003",
            "class": "bottle",
            "confidence": 0.78,
            "bbox": {"x_min": 50, "y_min": 300, "x_max": 200, "y_max": 450},
            "position_3d": {"x": -1.2, "y": -0.5, "z": 0.6},
            "color": "green"
        }
    ]

    # Test 1: Ground "red apple" to the red apple
    result1 = pipeline.ground_language_to_objects("red apple", mock_objects)
    print(f"Grounding 'red apple': {result1['class'] if result1 else None}")

    # Test 2: Ground "blue bottle" to the blue bottle
    result2 = pipeline.ground_language_to_objects("blue bottle", mock_objects)
    print(f"Grounding 'blue bottle': {result2['class'] if result2 else None}")

    # Test 3: Ground "left bottle" (should prefer the one at x=-1.2)
    result3 = pipeline.ground_language_to_objects("left bottle", mock_objects)
    print(f"Grounding 'left bottle': {result3['class'] if result3 else None} at x={result3['position_3d']['x'] if result3 else None}")

    # Test 4: Disambiguate between bottles
    bottles = [obj for obj in mock_objects if obj["class"] == "bottle"]
    disambiguated = pipeline.disambiguate_objects("blue bottle", bottles)
    print(f"Disambiguated bottles: {[obj['color'] for obj in disambiguated]}")

    # Test 5: Ground to multiple objects
    multiple_result = pipeline.ground_to_multiple_objects("bottle", mock_objects)
    print(f"Multiple bottle grounding: {len(multiple_result)} bottles found")

    # Test 6: Create grounding context
    context = pipeline.create_grounding_context("red apple", mock_objects)
    print(f"Grounding context created with {len(context['all_matches'])} matches")

    # Test 7: Validate grounding
    validation = pipeline.validate_grounding("red apple", result1)
    print(f"Validation result: valid={validation['valid']}, confidence={validation['confidence']:.3f}")

    # Check if tests passed appropriately
    success = (
        result1 and result1["class"] == "apple" and result1["color"] == "red" and  # Red apple should be found
        result2 and result2["class"] == "bottle" and result2["color"] == "blue" and  # Blue bottle should be found
        result3 and result3["class"] == "bottle" and result3["position_3d"]["x"] < 0 and  # Left bottle should be negative x
        len(multiple_result) == 2 and all(obj["class"] == "bottle" for obj in multiple_result) and  # Should find both bottles
        validation["valid"]  # Validation should pass
    )

    if success:
        print("✓ Grounding pipeline tests passed")
        return True
    else:
        print("✗ Grounding pipeline tests failed")
        return False


if __name__ == "__main__":
    test_grounding_pipeline()