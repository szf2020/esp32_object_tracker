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

# Function to handle client messages
async def handle_client(websocket, path):
    client_ip = websocket.remote_address[0]
    print(f"Client connected: {client_ip}")

    fig, ax = plt.subplots()  # Create the figure and axes once
    img_display = None  # To hold the image display object

    try:
        async for message in websocket:
            # Check if the message is binary (image data)
            if isinstance(message, bytes):
                print("Received image data from ESP32")

                try:
                    
                    image = Image.open(io.BytesIO(message)) # Convert JPEG binary data to a PIL Image
                    img_array = np.array(image) # Convert the PIL Image to a NumPy array

                    if img_display is None:
                        img_display = ax.imshow(img_array)
                    else:
                        img_display.set_data(img_array)

                    plt.draw()
                    plt.pause(0.01)  # Short pause to allow GUI updates

                except Exception as e:
                    print(f"Error decoding image: {e}")

                # Simulate object detection or image processing
                print("Processing image...")
                # (Run object detection or any other logic here)

                # Send movement command based on processed image (example)
                movement_command = "MOVE_LEFT"  # Example command
                await websocket.send(movement_command)
                print(f"Sent command to ESP32: {movement_command}")
            
            else:
                # Handle text messages or other non-binary messages
                print(f"Received message from ESP32: {message}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection with {client_ip} closed: {e}")


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # This doesn't have to be reachable; it's just used to obtain the correct interface
        s.connect(("8.8.8.8", 80))  # Connect to Google's public DNS
        ip = s.getsockname()[0]  # Get the local IP address used for the connection
    except Exception as e:
        ip = "Unable to get IP"
    finally:
        s.close()
    return ip

server_ip = get_local_ip()
print(f"Server IP address: {server_ip}")
# Start the WebSocket server
start_server = websockets.serve(handle_client, "0.0.0.0", 8765)

# Run the WebSocket server forever
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()