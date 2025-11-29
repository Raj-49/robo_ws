#!/usr/bin/env python3
"""
Object Detector - Color-based detection for pick and place
"""

import cv2
import numpy as np


class ColorObjectDetector:
    """Detect colored objects in RGB images"""
    
    def __init__(self):
        # HSV color ranges for different objects
        # Format: (lower_hsv, upper_hsv)
        self.color_ranges = {
            # Widened ranges for shadows (low V) and bright light (low S)
            # Lowered min Saturation and Value to 20 to catch very dark/washed out objects
            'red': ([0, 20, 20], [15, 255, 255]),
            'red2': ([165, 20, 20], [180, 255, 255]),
            'blue': ([90, 20, 20], [140, 255, 255]),
            'green': ([35, 20, 20], [85, 255, 255]),
            'yellow': ([15, 20, 20], [40, 255, 255]),
        }
        
        self.min_area = 100  # Minimum contour area (pixels)
        self.max_area = 5000 # Maximum contour area (pixels) - excludes baskets
    
    def detect(self, image, color='red'):
        """
        Detect object of specified color
        
        Args:
            image: BGR image from camera (numpy array)
            color: 'red', 'blue', 'green', or 'yellow'
        
        Returns:
            dict with:
              - 'center': (u, v) pixel coordinates of object center
              - 'bbox': (x, y, w, h) bounding box
              - 'area': contour area in pixels
            Returns None if not found
        """
        if image is None or image.size == 0:
            return None
        
    def get_mask(self, image, color='red'):
        """Get binary mask for specified color"""
        if image is None:
            return None
            
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if color == 'red':
            lower1, upper1 = self.color_ranges['red']
            lower2, upper2 = self.color_ranges['red2']
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower, upper = self.color_ranges.get(color, self.color_ranges['red'])
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask

    def detect(self, image, color='red'):
        """
        Detect object of specified color
        
        Args:
            image: BGR image from camera (numpy array)
            color: 'red', 'blue', 'green', or 'yellow'
        
        Returns:
            dict with:
              - 'center': (u, v) pixel coordinates of object center
              - 'bbox': (x, y, w, h) bounding box
              - 'area': contour area in pixels
            Returns None if not found
        """
        if image is None or image.size == 0:
            return None
        
        # Get mask
        mask = self.get_mask(image, color)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Filter by area
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if self.min_area < area < self.max_area:
                valid_contours.append(c)
        
        if not valid_contours:
            return None
        
        largest = max(valid_contours, key=cv2.contourArea)
        
        # Get centroid
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
        Detect all colored objects
        
        Returns:
            dict mapping color -> detection result
        """
        results = {}
        for color in ['red', 'blue', 'green', 'yellow']:
            detection = self.detect(image, color)
            if detection is not None:
                results[color] = detection
        return results
    
    def visualize(self, image, detections, color_name='red'):
        """
        Draw detection on image
        
        Args:
            image: BGR image
            detections: dict from detect() or detect_all()
            color_name: which color to visualize (if dict)
        
        Returns:
            Image with visualization
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
        
        # Draw center
        u, v = det['center']
        cv2.circle(vis, (u, v), 5, color_bgr, -1)
        cv2.circle(vis, (u, v), 10, color_bgr, 2)
        
        # Add label
        label = f"{color_name}: ({u}, {v})"
        cv2.putText(vis, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, color_bgr, 2)
        
        return vis

    def debug_hsv_at_center(self, image):
        """Print HSV value at image center for debugging"""
        if image is None:
            return
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pixel = hsv[cy, cx]
        print(f"DEBUG: HSV at center ({cx}, {cy}): {pixel}")


if __name__ == "__main__":
    # Test with sample image
    print("ColorObjectDetector initialized")
    print("Available colors:", ['red', 'blue', 'green', 'yellow'])
    detector = ColorObjectDetector()
    print("Ready for detection!")
