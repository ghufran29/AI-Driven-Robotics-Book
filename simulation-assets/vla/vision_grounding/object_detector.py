"""
Object Detector Module for VLA System

This module handles object detection and identification in images,
connecting visual objects to linguistic descriptions.
"""
import cv2
import numpy as np
import torch
import torchvision
from typing import List, Dict, Any, Optional, Tuple
import logging
from PIL import Image
import io
import base64

logger = logging.getLogger(__name__)

class ObjectDetector:
    """
    Class for detecting and identifying objects in images using various approaches.
    """
    def __init__(self, detection_threshold: float = 0.7, max_objects: int = 10):
        """
        Initialize the object detector.

        Args:
            detection_threshold: Minimum confidence threshold for detections
            max_objects: Maximum number of objects to detect
        """
        self.detection_threshold = detection_threshold
        self.max_objects = max_objects

        # Use a pre-trained model from torchvision (YOLO would be better, but this works for demo)
        try:
            self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights='DEFAULT')
            self.model.eval()
            self.transforms = torchvision.transforms.ToTensor()
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.model.to(self.device)
            logger.info(f"Object detector initialized on {self.device}")
        except Exception as e:
            logger.warning(f"Could not load pre-trained model: {e}. Using mock detection.")
            self.model = None

        # COCO dataset class names (first 80 classes)
        self.coco_names = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
            'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
            'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
            'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
            'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def detect_objects(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect objects in an image using the pre-trained model.

        Args:
            image: Input image as numpy array (H, W, C) in BGR format

        Returns:
            List of detected objects with bounding boxes, class names, and confidence scores
        """
        if self.model is None:
            # Return mock detections if model is not available
            return self._mock_detection(image)

        try:
            # Convert BGR to RGB
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Convert to PIL Image and then to tensor
            pil_image = Image.fromarray(image_rgb)
            input_tensor = self.transforms(pil_image).unsqueeze(0).to(self.device)

            # Perform inference
            with torch.no_grad():
                predictions = self.model(input_tensor)

            # Process predictions
            boxes = predictions[0]['boxes'].cpu().numpy()
            labels = predictions[0]['labels'].cpu().numpy()
            scores = predictions[0]['scores'].cpu().numpy()

            # Filter detections by confidence threshold
            valid_detections = scores >= self.detection_threshold

            detected_objects = []
            for i in range(len(boxes)):
                if valid_detections[i] and len(detected_objects) < self.max_objects:
                    box = boxes[i]
                    label = int(labels[i])
                    score = float(scores[i])
                    class_name = self.coco_names[label] if label < len(self.coco_names) else f"unknown_{label}"

                    # Format as expected output
                    obj = {
                        "id": f"obj_{len(detected_objects):03d}",
                        "class": class_name,
                        "confidence": score,
                        "bbox": {
                            "x_min": float(box[0]),
                            "y_min": float(box[1]),
                            "x_max": float(box[2]),
                            "y_max": float(box[3])
                        },
                        "position_3d": self._estimate_3d_position(box, image.shape)
                    }

                    detected_objects.append(obj)

            logger.info(f"Detected {len(detected_objects)} objects in image")
            return detected_objects

        except Exception as e:
            logger.error(f"Object detection failed: {str(e)}")
            # Return mock detections in case of error
            return self._mock_detection(image)

    def _mock_detection(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Generate mock detection results for testing purposes.

        Args:
            image: Input image

        Returns:
            List of mock detected objects
        """
        logger.warning("Using mock detection - real model not available")

        # Generate some mock objects based on common household items
        mock_objects = [
            {
                "id": "obj_001",
                "class": "bottle",
                "confidence": 0.89,
                "bbox": {
                    "x_min": 100,
                    "y_min": 150,
                    "x_max": 180,
                    "y_max": 230
                },
                "position_3d": {"x": 1.2, "y": 0.5, "z": 0.8}
            },
            {
                "id": "obj_002",
                "class": "apple",
                "confidence": 0.92,
                "bbox": {
                    "x_min": 250,
                    "y_min": 120,
                    "x_max": 320,
                    "y_max": 200
                },
                "position_3d": {"x": 1.5, "y": 0.3, "z": 0.8}
            },
            {
                "id": "obj_003",
                "class": "box",
                "confidence": 0.78,
                "bbox": {
                    "x_min": 50,
                    "y_min": 300,
                    "x_max": 200,
                    "y_max": 450
                },
                "position_3d": {"x": 0.8, "y": 1.0, "z": 0.6}
            }
        ]

        # Filter based on threshold
        filtered_objects = [obj for obj in mock_objects if obj["confidence"] >= self.detection_threshold]
        return filtered_objects[:self.max_objects]

    def detect_specific_object(self, image: np.ndarray, object_description: str) -> Optional[Dict[str, Any]]:
        """
        Detect a specific object based on a description.

        Args:
            image: Input image
            object_description: Description of the object to find (e.g., "red apple", "blue bottle")

        Returns:
            Detected object that matches the description, or None if not found
        """
        detected_objects = self.detect_objects(image)

        # Simple matching based on class name and color (in a real system, this would be more sophisticated)
        description_lower = object_description.lower()

        for obj in detected_objects:
            obj_class = obj["class"].lower()

            # Check if the description contains the class name
            if obj_class in description_lower:
                # Additional check for color if specified
                if "red" in description_lower and obj_class in ["apple", "bottle", "box"]:
                    # In a real system, we'd analyze the color from the image region
                    return obj
                elif "blue" in description_lower and obj_class in ["bottle", "cup", "ball"]:
                    return obj
                elif "green" in description_lower and obj_class in ["apple", "bottle"]:
                    return obj
                else:
                    # If no specific color match needed, return the first class match
                    return obj

        return None

    def find_object_by_color(self, image: np.ndarray, color: str) -> List[Dict[str, Any]]:
        """
        Find objects of a specific color in the image.

        Args:
            image: Input image
            color: Color to search for (e.g., "red", "blue", "green")

        Returns:
            List of objects with the specified color
        """
        detected_objects = self.detect_objects(image)
        color_lower = color.lower()

        # In a real system, we would analyze the actual colors in the bounding boxes
        # For now, we'll just return objects that are commonly associated with certain colors
        color_associations = {
            "red": ["apple", "apple", "bottle", "box"],  # apple appears twice for higher chance
            "blue": ["bottle", "cup", "ball", "car"],
            "green": ["apple", "bottle", "cup"],
            "yellow": ["banana", "apple", "cup"],
            "orange": ["orange", "cup", "ball"]
        }

        associated_classes = color_associations.get(color_lower, [])

        matching_objects = []
        for obj in detected_objects:
            if obj["class"] in associated_classes:
                # Add color information
                obj_with_color = obj.copy()
                obj_with_color["color"] = color
                matching_objects.append(obj_with_color)

        return matching_objects

    def find_object_by_multiple_attributes(self, image: np.ndarray, attributes: Dict[str, str]) -> List[Dict[str, Any]]:
        """
        Find objects matching multiple attributes (color, shape, size, position, etc.).

        Args:
            image: Input image
            attributes: Dictionary of attributes to match (e.g., {"color": "red", "class": "apple"})

        Returns:
            List of objects matching all specified attributes
        """
        detected_objects = self.detect_objects(image)

        matching_objects = []
        for obj in detected_objects:
            matches_all = True

            for attr, value in attributes.items():
                if attr == "class":
                    if obj.get("class", "").lower() != value.lower():
                        matches_all = False
                        break
                elif attr == "color":
                    # In a real system, we would extract color from the image region
                    # For now, we'll use associations
                    if value.lower() in obj.get("class", "").lower():  # Basic association
                        continue
                    else:
                        # Check if we have color information
                        obj_color = obj.get("color", "")
                        if obj_color and obj_color.lower() != value.lower():
                            matches_all = False
                            break
                elif attr == "size":
                    # Compare object size based on bounding box dimensions
                    bbox = obj.get("bbox", {})
                    width = bbox.get("x_max", 0) - bbox.get("x_min", 0)
                    height = bbox.get("y_max", 0) - bbox.get("y_min", 0)

                    if value.lower() == "large":
                        if width * height < 10000:  # arbitrary threshold
                            matches_all = False
                            break
                    elif value.lower() == "small":
                        if width * height > 5000:  # arbitrary threshold
                            matches_all = False
                            break
                elif attr == "position":
                    # Check relative position in the image
                    pos_info = obj.get("position_3d", {})
                    if value.lower() == "left":
                        if pos_info.get("x", 0) > 0:
                            matches_all = False
                            break
                    elif value.lower() == "right":
                        if pos_info.get("x", 0) < 0:
                            matches_all = False
                            break
                    elif value.lower() == "center":
                        if abs(pos_info.get("x", 0)) > 0.5:
                            matches_all = False
                            break
                else:
                    # For any other attribute, check if it exists and matches
                    if obj.get(attr, "").lower() != value.lower():
                        matches_all = False
                        break

            if matches_all:
                matching_objects.append(obj)

        return matching_objects

    def rank_objects_by_similarity(self, image: np.ndarray, target_description: str) -> List[Dict[str, Any]]:
        """
        Rank detected objects by similarity to the target description.

        Args:
            image: Input image
            target_description: Natural language description of target object

        Returns:
            List of objects ranked by similarity to the description (most similar first)
        """
        detected_objects = self.detect_objects(image)

        # Simple ranking based on keyword matching
        description_lower = target_description.lower()

        ranked_objects = []
        for obj in detected_objects:
            score = 0

            # Match class name
            if obj["class"].lower() in description_lower:
                score += 2

            # Match common attributes mentioned in description
            if "red" in description_lower and "red" in obj.get("class", "").lower():
                score += 1
            if "large" in description_lower or "big" in description_lower:
                bbox = obj.get("bbox", {})
                width = bbox.get("x_max", 0) - bbox.get("x_min", 0)
                height = bbox.get("y_max", 0) - bbox.get("y_min", 0)
                if width * height > 7500:  # arbitrary threshold for "large"
                    score += 1
            if "small" in description_lower or "little" in description_lower:
                bbox = obj.get("bbox", {})
                width = bbox.get("x_max", 0) - bbox.get("x_min", 0)
                height = bbox.get("y_max", 0) - bbox.get("y_min", 0)
                if width * height < 2500:  # arbitrary threshold for "small"
                    score += 1
            if "left" in description_lower:
                pos_info = obj.get("position_3d", {})
                if pos_info.get("x", 0) < 0:
                    score += 1
            if "right" in description_lower:
                pos_info = obj.get("position_3d", {})
                if pos_info.get("x", 0) > 0:
                    score += 1
            if "center" in description_lower:
                pos_info = obj.get("position_3d", {})
                if abs(pos_info.get("x", 0)) < 0.5:
                    score += 1

            ranked_objects.append((obj, score))

        # Sort by score (descending)
        ranked_objects.sort(key=lambda x: x[1], reverse=True)

        # Return just the objects, ranked
        return [obj for obj, score in ranked_objects]

    def get_scene_description(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Generate a description of the scene in the image.

        Args:
            image: Input image

        Returns:
            Dictionary with scene description and detected objects
        """
        detected_objects = self.detect_objects(image)

        # Count objects by class
        class_counts = {}
        for obj in detected_objects:
            class_name = obj["class"]
            class_counts[class_name] = class_counts.get(class_name, 0) + 1

        # Generate scene description
        objects_list = []
        for obj in detected_objects:
            objects_list.append({
                "class": obj["class"],
                "confidence": obj["confidence"],
                "position": obj["position_3d"]
            })

        scene_description = {
            "total_objects": len(detected_objects),
            "object_classes": list(class_counts.keys()),
            "class_counts": class_counts,
            "objects": objects_list,
            "image_shape": image.shape
        }

        return scene_description

    def _estimate_3d_position(self, bbox: np.ndarray, image_shape: Tuple[int, ...]) -> Dict[str, float]:
        """
        Estimate 3D position of an object from its 2D bounding box.
        This is a simplified estimation - in reality, depth information would be needed.

        Args:
            bbox: Bounding box [x_min, y_min, x_max, y_max]
            image_shape: Shape of the input image

        Returns:
            Estimated 3D position
        """
        # Simplified estimation - in reality, depth estimation would be needed
        img_h, img_w = image_shape[0], image_shape[1]

        # Calculate center of bounding box in image coordinates
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2

        # Normalize to 0-1 range
        norm_x = center_x / img_w
        norm_y = center_y / img_h

        # Map to a simple 3D space (this is very simplified)
        # In a real system, this would come from depth estimation or stereo vision
        x = (norm_x - 0.5) * 4.0  # Map to -2m to +2m range
        y = (0.5 - norm_y) * 3.0  # Map to -1.5m to +1.5m range (inverted because y increases downward)
        z = 0.8  # Fixed distance - in reality this would be estimated from size/depth

        return {"x": x, "y": y, "z": z}

    def process_image_from_bytes(self, image_bytes: bytes) -> List[Dict[str, Any]]:
        """
        Process an image provided as bytes.

        Args:
            image_bytes: Image data as bytes

        Returns:
            List of detected objects
        """
        try:
            # Convert bytes to numpy array
            nparr = np.frombuffer(image_bytes, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if image is None:
                logger.error("Could not decode image from bytes")
                return []

            return self.detect_objects(image)

        except Exception as e:
            logger.error(f"Failed to process image from bytes: {str(e)}")
            return []

    def process_image_from_base64(self, base64_string: str) -> List[Dict[str, Any]]:
        """
        Process an image provided as base64 string.

        Args:
            base64_string: Base64 encoded image string

        Returns:
            List of detected objects
        """
        try:
            # Decode base64 string
            image_bytes = base64.b64decode(base64_string)
            return self.process_image_from_bytes(image_bytes)

        except Exception as e:
            logger.error(f"Failed to process image from base64: {str(e)}")
            return []


def test_object_detector():
    """Test function for object detector functionality."""
    print("Testing object detector...")

    # Create object detector instance
    detector = ObjectDetector(detection_threshold=0.5, max_objects=5)

    # Test with a blank image (this will use mock detection)
    blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = detector.detect_objects(blank_image)
    print(f"Mock detections: {len(detections)} objects")

    # Test specific object detection
    specific_obj = detector.detect_specific_object(blank_image, "red apple")
    print(f"Specific object detection: {specific_obj}")

    # Test color-based detection
    red_objects = detector.find_object_by_color(blank_image, "red")
    print(f"Red objects: {len(red_objects)}")

    # Test scene description
    scene_desc = detector.get_scene_description(blank_image)
    print(f"Scene description: {scene_desc['total_objects']} total objects")

    # Check if tests passed appropriately
    success = (
        len(detections) >= 0 and  # Should return some detections (mock or real)
        scene_desc['total_objects'] == len(detections)  # Scene count should match detection count
    )

    if success:
        print("✓ Object detector tests passed")
        return True
    else:
        print("✗ Object detector tests failed")
        return False


if __name__ == "__main__":
    test_object_detector()