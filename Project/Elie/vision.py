import cv2

def get_image():
    webcam = cv2.VideoCapture('http://192.168.0.18:8080/video')
    check, frame = webcam.read()
    cv2.imwrite(filename='saved_img.jpg', img=frame)
    webcam.release()

# def filter_image():
    
def stream_video():
    #print("Before URL")
    cap = cv2.VideoCapture('http://192.168.0.18:8080/video')
    #print("After URL")

    while True:

        #print('About to start the Read command')
        ret, frame = cap.read()
        #print('About to show frame of Video.')
        cv2.imshow("Capturing",frame)
        #print('Running..')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    