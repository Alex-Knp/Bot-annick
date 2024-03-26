import cv2



def test_camera(camera_index=0):
    # Initialize the video capture object at the first camera index
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    
    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            # if frame is read correctly ret is True
        
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            
            # Display the resulting frame
            cv2.imshow('Camera Test', frame)
            
            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        # When everything is done, release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera()
