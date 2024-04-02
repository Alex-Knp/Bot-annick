import cv2
from picamera2 import Picamera2
import os

# Initialize Picamera2
picam2 = Picamera2()

# camera_config = picam2.create_video_configuration(main={"size":(640,480),"format":"RGB888"}, raw={"size": (640, 480)})
# picam2.configure(camera_config)

res_x = 640
res_y = 480
picam2.resolution = (res_x,res_y)
print("======================================================")
print(res_x,res_y)

picam2.start()  # Ensure you start the camera before capturing

directory = "/home/annick/Annick/Bot-annick/robot_code/camera/test_frames"

def empty_directory(directory):
    # Remove all files in the directory
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        try:
            os.remove(filepath)
        except Exception as e:
            print(f"Failed to delete {filepath}: {e}")

# Empty the directory before starting
empty_directory(directory)
print("test_frames emptied")

i = 0
try:
    while True:
        print(i)
        image = picam2.capture_array()  # Capture the image into a NumPy array
        image = image[:, :, [2, 1, 0]]    # RGB to BGR for opencv
        # cv2.imshow("Frame", image)  # Display the image

    
        # filename = os.path.join(directory, "frame{}.png".format(i))
        # cv2.imwrite(filename, image)
        filename = os.path.join(directory, f"frame{i}.png")
        picam2.capture(filename)

        i += 1
        if i == 100:
            break
        

        # Wait for 1 ms and check if the 'q' key is pressed to exit the loop
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
finally:
    cv2.destroyAllWindows()  # Cleanly close OpenCV windows
    picam2.stop()  # Stop the camera




