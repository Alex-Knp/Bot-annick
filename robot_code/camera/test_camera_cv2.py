import cv2

print("cv2 version : ",cv2.__version__)

def main():
    # Initialize the camera
    cap = cv2.VideoCapture(1)  # Change the parameter to 1 if using an external USB camera
    print('cap ok')
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Display the resulting frame
        
        cv2.imshow('Camera Stream', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
