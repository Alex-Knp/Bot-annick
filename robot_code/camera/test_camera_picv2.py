import cv2
from picamera2 import Picamera2
import os

# Initialize Picamera2
picam2 = Picamera2()
full_res = picam2.sensor_resolution

half_res=tuple([dim // 2 for dim in picam2.sensor_resolution])

print("full_res = ", full_res)
# camera_config = picam2.create_video_configuration(main={"size":(1920,1080),"format":"BGR888"}, raw={"size": (1920, 1080)})

# cam = picam2.configure(camera_config)
# print("cam_config : ", cam)
# picam2.configure(camera_config)

picam2.configure(picam2.create_video_configuration(raw={"size":half_res},main={"size": full_res})) 



print("========================================&============")


picam2.start()  # Ensure you start the camera before capturing



directory = "/home/annick/Annick/Bot-annick/robot_code/camera/test_frames"
dir2 = "/home/annick/Annick/Bot-annick/robot_code/camera/data/0"


def empty_directory(directory):
    # Remove all files in the directory
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        try:
            os.remove(filepath)
        except Exception as e:
            print(f"Failed to delete {filepath}: {e}")

# Empty the directory before starting
empty_directory(dir2)
print("directory emptied")

i = 0
try:
    while True:
        print(i)
        image = picam2.capture_array()  # Capture the image into a NumPy array
        image = image[:, :, [2, 1, 0]]    # RGB to BGR for opencv
        # cv2.imshow("Frame", image)  # Display the image

    
  
        filename = os.path.join(dir2, f"frame{i}.jpg")
        cv2.imwrite(filename, image)
        i += 1
        if i == 100:
            break
        # Wait for 1 ms and check if the 'q' key is pressed to exit the loop
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
finally:
    cv2.destroyAllWindows()  # Cleanly close OpenCV windows
    picam2.stop()  # Stop the camera




