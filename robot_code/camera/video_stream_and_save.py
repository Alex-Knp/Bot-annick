import cv2
import os

def main():
    # Create the frames directory if it doesn't exist
    frames_dir = 'frames'
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)

    # Initialize the camera
    cap = cv2.VideoCapture(0)  # Change the parameter to 1 if using an external USB camera

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    frame_count = 0
    fps = cap.get(cv2.CAP_PROP_FPS)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Save a frame every second
        if frame_count % int(fps) == 0:
            frame_filename = os.path.join(frames_dir, f'frame_{frame_count}.jpg')
            cv2.imwrite(frame_filename, frame)
            print(f"Saved frame {frame_count} as {frame_filename}")

        frame_count += 1

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
