from ultralytics import YOLO
import cv2

# Load a model
model = YOLO("yolov8x-pose.pt")  # load an official model

# Predict with the model
results = model("https://ultralytics.com/images/bus.jpg")  # predict on an image

annotated_frame = results[0].plot()  # Visualize the results on the frame (Everything)

cv2.imshow("annotated_frame", annotated_frame)

if cv2.waitKey(100000) & 0xFF == ord("q"):
    cv2.destroyAllWindows()