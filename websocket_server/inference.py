from collections import defaultdict

import cv2
import numpy as np

from ultralytics import YOLO


# Load the YOLOv8 model OBJECT DETECTION
model = YOLO("yolov8x.pt") # n,s,m,l,x

# FOR ESP32 CAM
stream_url = "http://192.168.86.139:81/stream"
cap = cv2.VideoCapture(stream_url)

# # FOR WEBCAM
# cap = cv2.VideoCapture(0)

# Store the track history
track_history = defaultdict(lambda: [])

# Check if the stream is opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        # results = model(frame)

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)

        # Get the boxes and track IDs
        clses = results[0].boxes.cls.cpu()
        boxes = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id
        if track_ids is not None:
            track_ids = track_ids.int().cpu().tolist()
        else:
            track_ids = []

        # Visualize the results on the frame
        annotated_frame = results[0].plot()


        # Plot the tracks
        for cls, box, track_id in zip(clses, boxes, track_ids):
            name = model.names[cls.int().item()]

            x, y, w, h = box
            track = track_history[track_id]
            track.append((float(x), float(y)))  # x, y center point
            if len(track) > 30:  # retain 30 tracks for 30 frames
                track.pop(0)

            print(f"{name}_{track_id}, x: {x}, y: {y}")

            # Draw the tracking lines
            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()