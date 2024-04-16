import cv2
import numpy as np
from picamera2 import Picamera2
import os
import tensorflow as tf
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input, decode_predictions
import time
import os
import sys

# print("tensorflow version: ",tf.__version__)
# Load the pre-trained MobileNetV2 model
model = tf.keras.applications.MobileNetV2(weights='imagenet', include_top=True)

# Initialize video capture

# Initialize Picamera2
picam2 = Picamera2()
full_res = picam2.sensor_resolution

half_res=tuple([dim // 2 for dim in picam2.sensor_resolution])

# camera_config = picam2.create_video_configuration(main={"size":(1920,1080),"format":"BGR888"}, raw={"size": (1920, 1080)})

# cam = picam2.configure(camera_config)
# print("cam_config : ", cam)
# picam2.configure(camera_config)

picam2.configure(picam2.create_video_configuration(raw={"size":half_res},main={"size": full_res})) 



print("========================================&============")


picam2.start()  # Ensure you start the camera before capturing



directory = "/home/annick/Annick/Bot-annick/robot_code/camera/test_frames"
dir2 = "/home/annick/Annick/Bot-annick/robot_code/camera/data/0"


# def empty_directory(directory):
#     # Remove all files in the directory
#     for filename in os.listdir(directory):
#         filepath = os.path.join(directory, filename)
#         try:
#             os.remove(filepath)
#         except Exception as e:
#             print(f"Failed to delete {filepath}: {e}")

# # Empty the directory before starting
# empty_directory(dir2)
# print("directory emptied")

# i = 0
try:
    start_time = time.time()  # Initialize start time

    while True:
        frame = picam2.capture_array()  # Capture the image into a NumPy array
        frame = frame[:, :, [2, 1, 0]]

        # Resize the frame to 224x224 for the model
        img = cv2.resize(frame, (224, 224))
        img_array = tf.keras.preprocessing.image.img_to_array(img)
        img_array = np.expand_dims(img_array, axis=0)
        img_array = preprocess_input(img_array)

        # get rid of the unneccesary prints from the model
        original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')

        # Make predictions
        predictions = model.predict(img_array)

        sys.stdout = original_stdout

        # Decode the predictions
        label = decode_predictions(predictions, top=1)[0][0][1]
        confidence = decode_predictions(predictions, top=1)[0][0][2]

        # Check if the label is related to plants
        plant_detected = "pot" in label or "vase" in label

        if plant_detected:
            display_text = f"Plant detected, confidence: {confidence:.2f}"
            if not continuous_detection:
                first_detection_time = time.time()  # Update the detection time
                continuous_detection = True
        else:
            display_text = "No plant detected"
            continuous_detection = False  # Reset the continuous detection flag

        # Calculate elapsed time
        elapsed_time = time.time() - start_time
        fps = 1 / elapsed_time
        start_time = time.time()
        # Update the display_text with label and average FPS
        print(display_text,f"Label: {label}, Confidence: {confidence:.2f},FPS: {fps:.4f}", end="\r")

        # Display the frame
        # cv2.putText(frame, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # cv2.imshow('Plant Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Release the capture
      cv2.destroyAllWindows()
