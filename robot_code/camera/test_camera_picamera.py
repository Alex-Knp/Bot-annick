from picamera import PiCamera
from time import sleep

def main():
    # Initialize the PiCamera
    camera = PiCamera()
    camera.resolution = (640, 480)  # Set the resolution of the camera
    camera.start_preview()  # Start the camera preview

    # Add a delay to ensure the camera preview is fully loaded
    sleep(2)

    try:
        # Keep the script running to maintain camera preview
        while True:
            pass
    except KeyboardInterrupt:
        # Stop the camera preview and release resources
        camera.stop_preview()

if __name__ == "__main__":
    main()
