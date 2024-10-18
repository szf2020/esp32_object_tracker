import websockets
import asyncio
import numpy as np
import matplotlib.pyplot as plt
import socket
from PIL import Image
import io

import cv2
from collections import defaultdict
from ultralytics import YOLO


## Params
CLASS_TRACKED = 0  # 0 is the class ID for human
CONF_THRESHOLD = 0.65  # Confidence threshold

model = YOLO("yolov8x.pt") # n,s,m,l,x

async def handle_client(websocket, path):
    client_ip = websocket.remote_address[0]
    print(f"Client connected: {client_ip}")

    track_history = defaultdict(lambda: [])

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                print("Received image data from ESP32")

                try:
                    # Decode the JPEG image
                    image = Image.open(io.BytesIO(message)) # Convert JPEG binary data to a PIL Image
                    img_array = np.array(image)
                    
                    # Inference
                    results = model.track(img_array, persist=True)
                    
                    # Detection Bounding Box Annotation
                    annotated_frame = results[0].plot() # Visualize the results on the frame (Everything)

                    # Selected Tracked Object Track Line
                    track_ids = results[0].boxes.id
                    boxes = results[0].boxes.xywh.cpu()
                    if track_ids is not None:
                        clses = results[0].boxes.cls.cpu()
                        conf = results[0].boxes.conf.cpu()
                        boxes = results[0].boxes.xywh.cpu()
                        track_ids = track_ids.int().cpu()
                    
                        # Filter out non-tracked detections
                        human_mask = clses == CLASS_TRACKED
                        conf_mask = conf > CONF_THRESHOLD
                        mask = human_mask & conf_mask
                        clses = clses[mask]
                        conf = conf[mask]
                        boxes = boxes[mask]
                        track_ids = track_ids[mask]

                        # Plot Track Lines
                        for cls, box, track_id in zip(clses, boxes, track_ids):
                            name = model.names[cls.int().item()]

                            x, y, w, h = box
                            track = track_history[track_id]
                            track.append((float(x), float(y)))  # x, y center point
                            if len(track) > 30:  # retain 30 tracks for 30 frames
                                track.pop(0)

                            print(f"{name}_{track_id}, x: {x}, y: {y}")

                            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

                    cv2.imshow("YOLOv8 Inference", annotated_frame)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                    # Largest Tracked Object
                    box_areas = boxes[:, 2] * boxes[:, 3]
                    if len(box_areas) == 0:
                        movement_command = "STAY"
                    else:
                        max_area_idx = box_areas.argmax(0)
                        x_largest, y_largest, w_largest, h_largest = boxes[max_area_idx]
                        print(f"x_largest: {x_largest}, y_largest: {y_largest}")
                        if x_largest < 0.45 * img_array.shape[1]:
                            movement_command = "MOVE_LEFT"
                        elif x_largest > 0.55 * img_array.shape[1]:
                            movement_command = "MOVE_RIGHT"

                    # Send Movement command
                    await websocket.send(movement_command)
                    print(f"Sent command to ESP32: {movement_command}")

                except Exception as e:
                    print(f"Error decoding image: {e}")
            
            else:
                # Other message types
                print(f"Received message from ESP32: {message}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection with {client_ip} closed: {e}")


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # Connect to Google's public DNS
        ip = s.getsockname()[0]  # Get the local IP address used for the connection
    except Exception as e:
        ip = "Unable to get IP"
    finally:
        s.close()
    return ip

server_ip = get_local_ip()
print(f"Server IP address: {server_ip}")
start_server = websockets.serve(handle_client, "0.0.0.0", 8765)

# Run the WebSocket server forever
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()