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

# HSV range for green tennis balls
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

# Tracker and state dictionaries
ct = CentroidTracker(maxDisappeared=30)
trail_dict = {}            # ID -> deque of points
color_dict = {}            # ID -> outline color
fill_color_dict = {}       # ID -> unique fill color
initial_centroids = {}     # ID -> initial (x, y)
movement_threshold = 20    # pixels

# Video input
if not args.get("video", False):
    vs = VideoStream(src=0).start()
else:
    vs = cv2.VideoCapture(args["video"])

time.sleep(2.0)

while True:
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame
    if frame is None:
        break

    # Preprocessing
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Masking
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None, iterations=2)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    inputCentroids = []
    contour_centroid_map = {}

    for c in cnts:
        if cv2.contourArea(c) < 150:
            continue

        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 10:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            inputCentroids.append((cX, cY))
            contour_centroid_map[(cX, cY)] = (c, (x, y), radius)

    # Update tracker
    objects = ct.update(inputCentroids)

    for objectID, centroid in objects.items():
        cX, cY = centroid
        trail = trail_dict.setdefault(objectID, deque(maxlen=args["buffer"]))
        outline_color = color_dict.setdefault(objectID, tuple(random.choices(range(100, 256), k=3)))

        # Assign a different fill color
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

        # Draw outline and fill (if moved)
        if centroid in contour_centroid_map:
            _, (x, y), radius = contour_centroid_map[centroid]
            center = (int(x), int(y))
            radius = int(radius)

            # Always draw outline
            cv2.circle(frame, center, radius, outline_color, 3)

            # Fill if moved
            if distance > movement_threshold:
                cv2.circle(frame, center, radius - 3, fill_color, -1)

        # Label
        cv2.putText(frame, f"ID {objectID}", (cX - 10, cY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, outline_color, 2)

        # Trail
        trail.appendleft(centroid)
        for i in range(1, len(trail)):
            if trail[i - 1] is None or trail[i] is None:
                continue
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, trail[i - 1], trail[i], outline_color, thickness)

    cv2.imshow("Multi-Ball Tracker", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Cleanup
if not args.get("video", False):
    vs.stop()
else:
    vs.release()
cv2.destroyAllWindows()