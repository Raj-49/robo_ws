#!/usr/bin/env python3
"""
Object Detector - Modular color-based detection for pick and place
Supports red, green, blue, and yellow object detection with configurable color sets.
"""

import cv2
import numpy as np


class ColorObjectDetector:
    """
    Modular color-based object detector supporting multiple colors.
    
    Features:
    - Configurable color set (red, green, blue, yellow)
    - HSV-based detection robust to lighting variations
    - Area filtering to exclude noise and large objects (baskets)
    - Morphological operations for noise reduction
    """
    
    def __init__(self, colors=None, min_area=100, max_area=5000):
        """
        Initialize color object detector.
        
        Args:
            colors: List of colors to detect ['red', 'green', 'blue', 'yellow'].
                   If None, detects all available colors.
            min_area: Minimum contour area in pixels (default: 100)
            max_area: Maximum contour area in pixels (default: 5000)
        """
        # HSV color ranges for different objects
        # Format: (lower_hsv, upper_hsv)
        # Widened ranges for shadows (low V) and bright light (low S)
        # Lowered min Saturation and Value to 20 to catch very dark/washed out objects
        self.color_ranges = {
            'red': ([0, 20, 20], [15, 255, 255]),
            'red2': ([165, 20, 20], [180, 255, 255]),  # Red wraps around in HSV
            'blue': ([90, 20, 20], [140, 255, 255]),
            'green': ([35, 20, 20], [85, 255, 255]),
            'yellow': ([15, 20, 20], [40, 255, 255]),
        }
        
        # Set which colors to detect
        self.active_colors = colors if colors is not None else ['red', 'green', 'blue', 'yellow']
        
        # Area filtering parameters
        self.min_area = min_area  # Minimum contour area (pixels) - excludes noise
        self.max_area = max_area  # Maximum contour area (pixels) - excludes baskets
    
    def get_mask(self, image, color='red'):
        """
        Generate binary mask for specified color.
        
        Args:
            image: BGR image from camera
            color: Color to detect ('red', 'green', 'blue', 'yellow')
        
        Returns:
            Binary mask (numpy array) where detected color is white (255)
        """
        if image is None:
            return None
            
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red requires special handling (wraps around in HSV)
        if color == 'red':
            lower1, upper1 = self.color_ranges['red']
            lower2, upper2 = self.color_ranges['red2']
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            if color not in self.color_ranges:
                return None
            lower, upper = self.color_ranges[color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill small holes
        
        return mask

    def detect(self, image, color='red'):
        """
        Detect object of specified color.
        
        Args:
            image: BGR image from camera (numpy array)
            color: 'red', 'blue', 'green', or 'yellow'
        
        Returns:
            dict with:
              - 'center': (u, v) pixel coordinates of object center
              - 'bbox': (x, y, w, h) bounding box
              - 'area': contour area in pixels
              - 'contour': largest contour (for visualization)
            Returns None if not found
        """
        if image is None or image.size == 0:
            return None
        
        # Get binary mask for color
        mask = self.get_mask(image, color)
        if mask is None:
            return None
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Filter by area (exclude noise and large objects like baskets)
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if self.min_area < area < self.max_area:
                valid_contours.append(c)
        
        if not valid_contours:
            return None
        
        # Get largest valid contour
        largest = max(valid_contours, key=cv2.contourArea)
        
        # Calculate centroid using image moments
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        
        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest)
        
        # Get area
        area = cv2.contourArea(largest)
        
        return {
            'center': (u, v),
            'bbox': (x, y, w, h),
            'area': area,
            'contour': largest
        }
    
    def detect_all(self, image):
        """
        Detect all configured colored objects.
        
        Args:
            image: BGR image from camera
        
        Returns:
            dict mapping color -> detection result
            Example: {'red': {...}, 'green': {...}}
        """
        results = {}
        for color in self.active_colors:
            detection = self.detect(image, color)
            if detection is not None:
                results[color] = detection
        return results
    
    def visualize(self, image, detections, color_name='red'):
        """
        Draw detection on image.
        
        Args:
            image: BGR image
            detections: dict from detect() or detect_all()
            color_name: which color to visualize (if dict)
        
        Returns:
            Image with visualization (bounding box, center, label)
        """
        vis = image.copy()
        
        # Handle single detection or dict
        if isinstance(detections, dict):
            if 'center' in detections:
                # Single detection
                det = detections
                color_bgr = (0, 0, 255)  # Red
            else:
                # Multiple detections
                if color_name not in detections:
                    return vis
                det = detections[color_name]
                color_map = {
                    'red': (0, 0, 255),
                    'blue': (255, 0, 0),
                    'green': (0, 255, 0),
                    'yellow': (0, 255, 255)
                }
                color_bgr = color_map.get(color_name, (0, 0, 255))
        else:
            return vis
        
        # Draw bounding box
        x, y, w, h = det['bbox']
        cv2.rectangle(vis, (x, y), (x + w, y + h), color_bgr, 2)
        
        # Draw center point
        u, v = det['center']
        cv2.circle(vis, (u, v), 5, color_bgr, -1)
        cv2.circle(vis, (u, v), 10, color_bgr, 2)
        
        # Add label with position
        label = f"{color_name}: ({u}, {v})"
        cv2.putText(vis, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, color_bgr, 2)
        
        return vis

    def debug_hsv_at_center(self, image):
        """Print HSV value at image center for debugging color ranges."""
        if image is None:
            return
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pixel = hsv[cy, cx]
        print(f"DEBUG: HSV at center ({cx}, {cy}): {pixel}")


if __name__ == "__main__":
    # Test initialization
    print("ColorObjectDetector - Modular Color Detection")
    print("=" * 50)
    
    # Example 1: Detect all colors
    detector_all = ColorObjectDetector()
    print(f"Detector 1 - All colors: {detector_all.active_colors}")
    
    # Example 2: Detect only red and green
    detector_rg = ColorObjectDetector(colors=['red', 'green'])
    print(f"Detector 2 - Red/Green only: {detector_rg.active_colors}")
    
    # Example 3: Custom area thresholds
    detector_custom = ColorObjectDetector(colors=['blue'], min_area=200, max_area=3000)
    print(f"Detector 3 - Blue with custom areas: min={detector_custom.min_area}, max={detector_custom.max_area}")
    
    print("\nReady for detection!")
