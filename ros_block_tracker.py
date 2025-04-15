#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque
from centroid_tracker import CentroidTracker
import numpy as np
import cv2
import imutils
import time
import random

class BlockTracker:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('block_tracker', anonymous=True)
        
        # Setup CV bridge
        self.bridge = CvBridge()
        
        # Setup subscriber
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        
        # Initialize tracker
        self.ct = CentroidTracker(maxDisappeared=20)
        self.trail_dict = {}            # ID -> deque of points
        self.color_dict = {}           # ID -> outline color
        self.fill_color_dict = {}      # ID -> unique fill color
        self.initial_centroids = {}    # ID -> initial (x, y)
        self.movement_threshold = 8    # Movement detection threshold
        
        # Get parameters
        self.threshold = rospy.get_param('~threshold', 127)
        self.MIN_BLOCK_AREA = rospy.get_param('~min_block_area', 400)
        self.MAX_BLOCK_AREA = rospy.get_param('~max_block_area', 2500)
        self.ASPECT_RATIO_TOLERANCE = 0.3
        self.visualize = rospy.get_param('~visualize', True)
        
        rospy.loginfo("Block tracker initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Preprocessing
            frame = imutils.resize(frame, width=800)
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            
            # Convert to grayscale
            gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
            
            # Apply threshold
            _, thresh = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
            
            # Clean up thresholded image
            kernel = np.ones((3,3), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            # Find contours
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            inputCentroids = []
            contour_centroid_map = {}
            
            # Process contours
            for c in cnts:
                # Area check
                area = cv2.contourArea(c)
                if area < self.MIN_BLOCK_AREA or area > self.MAX_BLOCK_AREA:
                    continue
                
                # Shape check
                x, y, w, h = cv2.boundingRect(c)
                aspect_ratio = float(w)/h
                if abs(aspect_ratio - 1.0) > self.ASPECT_RATIO_TOLERANCE:
                    continue
                
                # Get centroid
                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue
                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                inputCentroids.append((cX, cY))
                contour_centroid_map[(cX, cY)] = c
            
            # Update tracker
            objects = self.ct.update(inputCentroids)
            
            # Process tracked objects
            for objectID, centroid in objects.items():
                cX, cY = centroid
                
                # Initialize tracking for new objects
                if objectID not in self.trail_dict:
                    self.trail_dict[objectID] = deque(maxlen=64)
                    self.color_dict[objectID] = tuple(random.choices(range(100, 256), k=3))
                    self.initial_centroids[objectID] = centroid
                
                # Calculate movement
                initial_centroid = self.initial_centroids[objectID]
                dx = cX - initial_centroid[0]
                dy = cY - initial_centroid[1]
                distance = np.sqrt(dx**2 + dy**2)
                is_moving = distance > self.movement_threshold
                
                # Log block information
                rospy.loginfo(f"Block {objectID} at ({cX}, {cY}) - Moving: {is_moving}")
                
                # Update trail
                self.trail_dict[objectID].appendleft(centroid)
                
                # Visualize if enabled
                if self.visualize:
                    self.visualize_block(frame, objectID, centroid, contour_centroid_map)
            
            # Show visualization if enabled
            if self.visualize:
                cv2.imshow("Block Tracker", frame)
                cv2.waitKey(1)
                
        except Exception as e:
            rospy.logerr(f"Error in image callback: {str(e)}")

    def visualize_block(self, frame, objectID, centroid, contour_centroid_map):
        """Visualize block tracking results"""
        cX, cY = centroid
        outline_color = self.color_dict[objectID]
        
        # Draw contour
        if centroid in contour_centroid_map:
            cv2.drawContours(frame, [contour_centroid_map[centroid]], -1, outline_color, 2)
        
        # Draw ID
        cv2.putText(frame, f"ID {objectID}", (cX - 10, cY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, outline_color, 2)
        
        # Draw trail
        trail = self.trail_dict[objectID]
        for i in range(1, len(trail)):
            if trail[i - 1] is None or trail[i] is None:
                continue
            thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
            cv2.line(frame, trail[i - 1], trail[i], outline_color, thickness)

def main():
    try:
        tracker = BlockTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 