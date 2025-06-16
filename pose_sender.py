import cv2
import mediapipe as mp
import socket
import json
import numpy as np

# --- MediaPipe Pose Setup ---
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=1,  # 0 for lite, 1 for full, 2 for heavy
    smooth_landmarks=True
)
mp_drawing = mp.solutions.drawing_utils

# --- UDP Socket Setup ---
UDP_IP = "127.0.0.1"  # Standard loopback interface address (localhost)
UDP_PORT = 5052       # Port to listen on (non-privileged ports are > 1023)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

# --- Webcam Setup ---
cap = cv2.VideoCapture(0) # 0 for default webcam
if not cap.isOpened():
    print("Cannot open camera")
    exit()

print(f"Sending pose data to {UDP_IP}:{UDP_PORT}")

try:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip the image horizontally for a later selfie-view display
        # And convert the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = pose.process(image)
        image.flags.writeable = True

        # Convert the image back to BGR for OpenCV.
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        landmarks_data = []
        if results.pose_landmarks:
            # Draw the pose annotation on the image.
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
            )

            # Extract landmarks
            for id, lm in enumerate(results.pose_landmarks.landmark):
                # Store landmark name, x, y, z, and visibility
                # x, y are normalized to [0.0, 1.0] by image width/height
                # z represents depth, with the hip joint as the origin. Smaller z = closer.
                landmarks_data.append({
                    "id": id,
                    "name": mp_pose.PoseLandmark(id).name,
                    "x": lm.x,
                    "y": lm.y,
                    "z": lm.z,
                    "visibility": lm.visibility
                })

            # Send data if landmarks were detected
            if landmarks_data:
                message = json.dumps(landmarks_data).encode('utf-8')
                sock.sendto(message, (UDP_IP, UDP_PORT))
                # print(f"Sent {len(landmarks_data)} landmarks. Sample: {landmarks_data[0]['name']}")
                #print(f"Sent {len(landmarks_data)} landmarks.")

        cv2.imshow('MediaPipe Pose', image)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
finally:
    print("Stopping pose sender.")
    cap.release()
    cv2.destroyAllWindows()
    pose.close()
    sock.close()