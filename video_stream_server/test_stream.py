import cv2

# Replace with your RTSP source
rtsp_source = 'rtmp://10.24.150.155:1935/TEST'

cap = cv2.VideoCapture(rtsp_source)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
