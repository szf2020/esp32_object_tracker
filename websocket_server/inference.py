from collections import defaultdict

import cv2
import numpy as np

from ultralytics import YOLO

## Params
CLASS_TRACKED = [0] # 0 is the class ID for human
CONF_THRESHOLD = 0.65  # Confidence threshold

# Load the YOLOv8 model OBJECT DETECTION
model = YOLO("yolov8x-pose.pt") # n,s,m,l,x

# # FOR ESP32 CAM
# stream_url = "http://192.168.86.139:81/stream"
# cap = cv2.VideoCapture(stream_url)

# FOR WEBCAM
cap = cv2.VideoCapture(0)

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

        # Inference
        results = model.track(frame, persist=True, conf=CONF_THRESHOLD, classes=CLASS_TRACKED)

        # Detection Bounding Box Annotation
        annotated_frame = results[0].plot() # Visualize the results on the frame (Everything)

        # Selected Tracked Object Track Line
        track_ids = results[0].boxes.id
        if track_ids is not None:
            track_ids = track_ids.int().cpu()
            kypts = results[0].keypoints.data.cpu()
            eye_dist = np.array([])

            # Plot Track Lines
            for kypt, track_id in zip(kypts, track_ids):
                track_id = track_id.item()
                nose = kypt[0]
                left_eye = kypt[1]
                right_eye = kypt[2]
                if left_eye[2] > CONF_THRESHOLD and right_eye[2] > CONF_THRESHOLD:
                    eye_dist = np.append(eye_dist, np.linalg.norm(left_eye[:2] - right_eye[:2]))
                else:
                    eye_dist = np.append(eye_dist, -1)

                nose_x, nose_y, nose_conf = nose[0], nose[1], nose[2]
                if nose_conf > CONF_THRESHOLD:
                    track = track_history[track_id]
                    track.append((float(nose_x), float(nose_y)))  # x, y center point

                    if len(track) > 30:  # retain 30 tracks for 30 frames
                        track.pop(0)

                    print(f"track_{track_id}, x: {nose_x}, y: {nose_y}, conf: {nose_conf}")

                    points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                    cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
                else:
                    print("Low Nose Confidence")
        else:
            print("No tracked objects")

        cv2.imshow("YOLOv8 Inference", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        # Largest Tracked Object
        if len(eye_dist) == 0:
            movement_command = "STAY"
        else:
            max_eye_dist_idx = eye_dist.argmax(0)
            x_largest, y_largest, conf_largest = kypts[max_eye_dist_idx][0]
            print(f"x_largest: {x_largest}, y_largest: {y_largest}")
            if x_largest < 0.45 * frame.shape[1]:
                movement_command = "MOVE_LEFT"
            elif x_largest > 0.55 * frame.shape[1]:
                movement_command = "MOVE_RIGHT"
            else:
                movement_command = "STAY"

        print(f"movement_command: {movement_command}")

    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()