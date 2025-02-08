import cv2
import numpy as np

# Load YOLOv4 model configuration and weights
yolo_cfg = "yolo/yolov4.cfg"  # Path to the YOLOv4 config file
yolo_weights = "yolo/yolov4.weights"  # Path to the YOLOv4 weights file
yolo_names = "yolo/coco.names"  # Path to the COCO class names file

# Load YOLO network
net = cv2.dnn.readNet(yolo_weights, yolo_cfg)

# Load COCO class labels
with open(yolo_names, "r") as f:
    classes = f.read().strip().split("\n")

# Get output layer names
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Get the class ID for "person"
person_class_id = classes.index("person")

def detect_person_in_frame():
    # Define the camera stream URL and changed with the url from clodflare or ngork
    stream_url = "https://institution-trial-interracial-receives.trycloudflare.com/stream?topic=/camera_sensor/image_raw"
    cap = cv2.VideoCapture(stream_url)

    if not cap.isOpened():
        print("Error: Cannot access the camera stream.")
        return None

    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame from the stream.")
        cap.release()
        return None

    try:
        # Process the frame (Person detection)
        height, width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)

        net.setInput(blob)
        layer_outputs = net.forward(output_layers)

        boxes = []
        confidences = []
        class_ids = []

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5 and class_id == person_class_id:
                    center_x, center_y, w, h = (detection[0:4] * np.array([width, height, width, height])).astype("int")
                    x = int(center_x - (w / 2))
                    y = int(center_y - (h / 2))
                    boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Maximum Suppression (NMS)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Draw bounding boxes for detected persons
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"Person {confidences[i]:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    except Exception as e:
        print(f"Error during person detection: {e}")
        frame = None

    # Release the video stream
    cap.release()
    return frame
