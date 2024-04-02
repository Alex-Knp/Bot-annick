import os

def capture_frame(frame_count):
    # Command to capture an image using libcamera-still
    command = f'libcamera-still -o images/test_{frame_count}.jpg'
    
    # Execute the command
    os.system(command)
    
    # Increment frame count
    frame_count += 1
    
    # Recursively call capture_frame with updated frame count
    capture_frame(frame_count)

if __name__ == "__main__":
    # Create directory to store images if it doesn't exist
    if not os.path.exists('images'):
        os.makedirs('images')
    
    # Start capturing frames
    capture_frame(0)
