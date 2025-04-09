from collections import deque
from imutils.video import VideoStream
from centroid_tracker import CentroidTracker
import numpy as np
import argparse
import cv2
import imutils
import time
import random

# Argument parsing
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to optional video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max trail length")
args = vars(ap.parse_args())

# Fine-tuned color ranges for counting blocks
# HSV ranges carefully selected for the specific colors shown in the image
color_ranges = {
    'red': [(0, 150, 150), (10, 255, 255)],      # Bright red range 1
    'red2': [(170, 150, 150), (180, 255, 255)],  # Bright red range 2
    'orange': [(8, 150, 150), (20, 255, 255)],   # Vibrant orange
    'yellow': [(20, 150, 150), (35, 255, 255)],  # Bright yellow
    'green': [(35, 100, 150), (85, 255, 255)],   # Lime green
    'blue': [(100, 150, 150), (130, 255, 255)],  # Bright blue
    'purple': [(130, 100, 150), (160, 255, 255)] # Vibrant purple
}

# Tracker and state dictionaries
ct = CentroidTracker(maxDisappeared=20)  # Reduced disappearance threshold
trail_dict = {}            # ID -> deque of points
color_dict = {}           # ID -> outline color
fill_color_dict = {}      # ID -> unique fill color
initial_centroids = {}    # ID -> initial (x, y)
movement_threshold = 8    # Reduced for more sensitive movement detection

# Video input
if not args.get("video", False):
    vs = VideoStream(src=0).start()
else:
    vs = cv2.VideoCapture(args["video"])

time.sleep(2.0)

# Define block size range (in pixels after resize)
MIN_BLOCK_AREA = 400  # Minimum area for a block
MAX_BLOCK_AREA = 2500  # Maximum area for a block
ASPECT_RATIO_TOLERANCE = 0.3  # How much deviation from square shape we allow

while True:
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame
    if frame is None:
        break

    # Preprocessing
    frame = imutils.resize(frame, width=800)  # Increased resolution
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)  # Reduced blur for sharper edges
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Combined mask for all colors
    combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    
    # Process each color range
    for color_name, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, lower, upper)
        if color_name == 'red':  # Special handling for red
            mask2 = cv2.inRange(hsv, color_ranges['red2'][0], color_ranges['red2'][1])
            mask = cv2.bitwise_or(mask, mask2)
        
        # Enhanced mask cleanup
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        combined_mask = cv2.bitwise_or(combined_mask, mask)

    # Find contours
    cnts = cv2.findContours(combined_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    inputCentroids = []
    contour_centroid_map = {}

    for c in cnts:
        # Calculate area and check if it's within our expected range
        area = cv2.contourArea(c)
        if area < MIN_BLOCK_AREA or area > MAX_BLOCK_AREA:
            continue

        # Check if the shape is approximately square
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = float(w)/h
        if abs(aspect_ratio - 1.0) > ASPECT_RATIO_TOLERANCE:
            continue

        # Use rotated rectangle for better fit
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        # Calculate centroid
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Store centroid and contour info
        inputCentroids.append((cX, cY))
        contour_centroid_map[(cX, cY)] = (c, box)

    # Update tracker
    objects = ct.update(inputCentroids)

    for objectID, centroid in objects.items():
        cX, cY = centroid
        trail = trail_dict.setdefault(objectID, deque(maxlen=args["buffer"]))
        outline_color = color_dict.setdefault(objectID, tuple(random.choices(range(100, 256), k=3)))

        if objectID not in fill_color_dict:
            while True:
                fill_color = tuple(random.choices(range(50, 256), k=3))
                if fill_color != outline_color:
                    break
            fill_color_dict[objectID] = fill_color

        fill_color = fill_color_dict[objectID]
        initial_centroid = initial_centroids.setdefault(objectID, centroid)

        # Calculate movement distance
        dx = cX - initial_centroid[0]
        dy = cY - initial_centroid[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Draw rectangle and fill
        if centroid in contour_centroid_map:
            _, box = contour_centroid_map[centroid]
            
            # Draw outline
            cv2.drawContours(frame, [box], 0, outline_color, 2)
            
            # Fill if moved
            if distance > movement_threshold:
                cv2.fillPoly(frame, [box], fill_color)

        # Label with smaller font and better positioning
        cv2.putText(frame, f"{objectID}", (cX - 8, cY + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, outline_color, 1)

        # Trail with thinner lines
        trail.appendleft(centroid)
        for i in range(1, len(trail)):
            if trail[i - 1] is None or trail[i] is None:
                continue
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 1.5)
            cv2.line(frame, trail[i - 1], trail[i], outline_color, thickness)

    # Add color detection indicator
    y_offset = 30
    for color_name, (lower, upper) in color_ranges.items():
        if color_name != 'red2':  # Skip second red range in display
            cv2.putText(frame, f"{color_name.capitalize()}", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 20

    cv2.imshow("Block Tracker", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Cleanup
if not args.get("video", False):
    vs.stop()
else:
    vs.release()
cv2.destroyAllWindows() 