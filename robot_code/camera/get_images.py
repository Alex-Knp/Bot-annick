# import cv2
# from picamera2 import Picamera2
# import os
# from termcolor import colored
# import random
# import re  # Import regex module to search for patterns in filenames
# import time  # Import time module for introducing delays

# # Initialize Picamera2
# picam2 = Picamera2()
# res = tuple([dim // 4 for dim in picam2.sensor_resolution])  # Low resolution
# x_res, y_res = res
# picam2.configure(picam2.create_video_configuration(raw={"size": res}, main={"size": res}))
# picam2.start()  # Start the camera before capturing
# print("====================================================")

# nb_frames = 1000
# category = "1_plant"

# def empty_directory(directory):
#     # Remove all files in the directory
#     for filename in os.listdir(directory):
#         filepath = os.path.join(directory, filename)
#         try:
#             os.remove(filepath)
#         except Exception as e:
#             print(f"Failed to delete {filepath}: {e}")

# def get_next_filename(directory, category, x_res, y_res):
#     pattern = re.compile(f"{category}_([0-9]+)_{x_res}x{y_res}.jpg")
#     max_index = 0
#     for filename in os.listdir(directory):
#         match = pattern.match(filename)
#         if match:
#             index = int(match.group(1))
#             if index > max_index:
#                 max_index = index
#     # Return the next filename with the incremented index
#     return f"{category}_{max_index + 1}_{x_res}x{y_res}.jpg"

# dir_train = "/home/annick/Annick/Bot-annick/robot_code/camera/data/train"
# dir_val = "/home/annick/Annick/Bot-annick/robot_code/camera/data/val"
# dir_test = "/home/annick/Annick/Bot-annick/robot_code/camera/data/test"

# clean = str(input("clear directories ? (y/n) "))
# if clean == "y":
#     empty_directory(dir_train)
#     empty_directory(dir_val)
#     empty_directory(dir_test)
#     print("directories emptied")

# directories = [dir_train] * int(nb_frames * 0.7) + [dir_val] * int(nb_frames * 0.2) + [dir_test] * int(nb_frames * 0.1)
# random.shuffle(directories)

# try:
#     start_time = time.time()
#     for i, dir_current in enumerate(directories):
#         print(i, end="\r")
#         image = picam2.capture_array()  # Capture the image into a NumPy array
#         image = image[:, :, [2, 1, 0]]  # Convert RGB to BGR for OpenCV

#         # Determine the filename based on existing files to avoid overwrites
#         filename = get_next_filename(dir_current, category, x_res, y_res)
#         full_path = os.path.join(dir_current, filename)
#         cv2.imwrite(full_path, image)

#         # Introduce a delay to capture 4 images per second
#         elapsed_time = time.time() - start_time
#         if elapsed_time < 0.1:
#             time.sleep(0.1 - elapsed_time)
#         start_time = time.time()
# except Exception as e:
#     print(f"An error occurred: {e}")
# finally:
#     print(colored('Training set complete', 'green'))
#     print(colored('Validation set complete', 'green'))
#     print(colored('Test set complete', 'green'))
#     picam2.stop()



import cv2
from picamera2 import Picamera2
import os
from termcolor import colored
import random

# Initialize Picamera2
picam2 = Picamera2()
res = tuple([dim // 4 for dim in picam2.sensor_resolution])  # Low resolution
x_res, y_res = res
picam2.configure(picam2.create_video_configuration(raw={"size": res}, main={"size": res}))
picam2.start()  # Start the camera before capturing
print("====================================================")

nb_frames = 1000
category = "2_plant"
databatch = "data4"


def empty_directory(directory):
    # Remove all files in the directory
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        try:
            os.remove(filepath)
        except Exception as e:
            print(f"Failed to delete {filepath}: {e}")

dir_train = "/home/annick/Annick/Bot-annick/robot_code/camera/data/train"
dir_val = "/home/annick/Annick/Bot-annick/robot_code/camera/data/val"
dir_test = "/home/annick/Annick/Bot-annick/robot_code/camera/data/test"

clean = str(input("clear directories ? (y/n) "))
if clean == "y":
    empty_directory(dir_train)
    empty_directory(dir_val)
    empty_directory(dir_test)
    print("directories emptied")

directories = [dir_train] * int(nb_frames * 0.7) + [dir_val] * int(nb_frames * 0.2) + [dir_test] * int(nb_frames * 0.1)
random.shuffle(directories)


try:
    for i, dir_current in enumerate(directories):
        print(i, end="\r")
        image = picam2.capture_array()  # Capture the image into a NumPy array
        image = image[:, :, [2, 1, 0]]  # Convert RGB to BGR for OpenCV

        # Name and save the image file
        filename = os.path.join(dir_current, f"{category}_{i}_{x_res}x{y_res}_{databatch}.jpg")
        cv2.imwrite(filename, image)
except Exception as e:
    print(f"An error occurred: {e}")

finally:
    print(colored('Training set complete', 'green'))
    print(colored('Validation set complete', 'green'))
    print(colored('Test set complete', 'green'))
    picam2.stop()


#install pynput

# import cv2
# from picamera2 import Picamera2
# import os
# from termcolor import colored
# import random
# from pynput import keyboard

# # Initialize Picamera2
# picam2 = Picamera2()
# res = tuple([dim // 4 for dim in picam2.sensor_resolution])  # Low resolution
# x_res, y_res = res
# picam2.configure(picam2.create_video_configuration(raw={"size": res}, main={"size": res}))
# picam2.start()  # Start the camera before capturing
# print("====================================================")

# nb_frames = 1000
# category = "1_plant"

# def empty_directory(directory):
#     # Remove all files in the directory
#     for filename in os.listdir(directory):
#         filepath = os.path.join(directory, filename)
#         try:
#             os.remove(filepath)
#         except Exception as e:
#             print(f"Failed to delete {filepath}: {e}")

# # Define global pause flag
# global is_paused
# is_paused = False

# # The key press event handler
# def on_press(key):
#     global is_paused
#     try:
#         if key.char == 'p':  # Assuming 'p' to pause and resume
#             is_paused = not is_paused
#             if is_paused:
#                 print("\nPaused. Press 'p' again to resume.")
#             else:
#                 print("\nResuming...")
#     except AttributeError:
#         pass  # Handle non-character keys

# # Start listening for the keyboard events
# listener = keyboard.Listener(on_press=on_press)
# listener.start()

# dir_train = "/home/annick/Annick/Bot-annick/robot_code/camera/data/train"
# dir_val = "/home/annick/Annick/Bot-annick/robot_code/camera/data/val"
# dir_test = "/home/annick/Annick/Bot-annick/robot_code/camera/data/test"

# # Your existing code to clear directories and setup continues here...

# try:
#     for i, dir_current in enumerate(directories):
#         if i >= nb_frames:  # Break the loop if the desired number of frames is reached
#             break
#         while is_paused:  # Wait here if paused
#             continue
        
#         print(i, end="\r")
#         image = picam2.capture_array()
#         image = image[:, :, [2, 1, 0]]  # Convert RGB to BGR for OpenCV

#         filename = os.path.join(dir_current, f"{category}_{i}_{x_res}x{y_res}.jpg")
#         cv2.imwrite(filename, image)
        
# except Exception as e:
#     print(f"An error occurred: {e}")
# finally:
#     print(colored('Training set complete', 'green'))
#     print(colored('Validation set complete', 'green'))
#     print(colored('Test set complete', 'green'))
#     picam2.stop()
#     listener.stop()  # Stop listening for keyboard events
