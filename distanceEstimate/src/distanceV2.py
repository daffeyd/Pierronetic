import numpy as np
import cv2

# Constants for known distance, width and colors
KNOWN_DISTANCE = 68  # centimeter
KNOWN_WIDTH = 14.5  # centimeter
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
fonts = cv2.FONT_HERSHEY_COMPLEX

# Load face detection model
faceNet = cv2.dnn.readNet("face_detector/deploy.prototxt", "face_detector/res10_300x300_ssd_iter_140000.caffemodel")

def calculate_focal_length(measured_distance, real_width, width_in_rf_image):
    """
    Function to calculate the focal length given a measured distance, a real width, and the width in the image
    """
    return (width_in_rf_image * measured_distance) / real_width

def estimate_distance(focal_length, real_face_width, face_width_in_frame):
    """
    Function to estimate the distance based on the focal length, real face width and the face width in the frame
    """
    return (real_face_width * focal_length) / face_width_in_frame if face_width_in_frame != 0 else 0

def detect_face(frame):
    """
    Function to detect face in the frame, draw a rectangle around it and return the face width
    """
    face_width = 0
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (224, 224), (104.0, 177.0, 123.0))
    faceNet.setInput(blob)
    detections = faceNet.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))
            face_width = endX - startX
            cv2.rectangle(frame, (startX, startY), (endX, endY), GREEN, 2)

    return face_width

def main():
    """
    Main function to read the reference image, calculate the focal length and then estimate distance in the frame
    """
    cap = cv2.VideoCapture(1)
    ref_image = cv2.imread("Ref_image.jpg")
    ref_image_face_width = detect_face(ref_image)
    focal_length_found = calculate_focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)
    print(f"Focal Length: {focal_length_found}")
    # cv2.imshow("Reference Image", ref_image)

    while True:
        _, frame = cap.read()
        face_width_in_frame = detect_face(frame)

        if face_width_in_frame != 0:
            distance = estimate_distance(focal_length_found, KNOWN_WIDTH, face_width_in_frame)
            cv2.putText(frame, f"Distance = {round(distance, 2)} CM", (50, 50), fonts, 1, WHITE, 2)

        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
