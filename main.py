import threading
import cv2
import mediapipe as mp
import numpy as np
import pyautogui
from threading import Thread
from time import sleep

# Reduce the delay
pyautogui.PAUSE = 0.01

moves = ['CENTER','CENTER']

def threaded_function(key):
    pyautogui.press(key)


# Initialize camera to default webcam camera
cam = cv2.VideoCapture(0)

# Create face_mesh using mediapipe
face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)
# Create hand_mesh using mediapipe
hand_mesh = mp.solutions.hands.Hands(max_num_hands=1)

# Threshold values for control orientations fine_tuning
lr_treshold = 20
ud_treshold = 15

delay = 0

# Display the webcam feed
while True and cam.isOpened():
    # Left-Right Orientation Control
    lr_control = "CENTER"
    # Up-Down Orientation Control
    ud_control = "CENTER"
    # Action-Control
    action_control = "NONE"

    # Read the webcam feed
    ret, frame = cam.read()

    # Convert the frame to RGB for processing
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Get face from webcam feed using face_mesh
    face = face_mesh.process(rgb_frame)
    # Get face landmark points from face
    face_landmarks = face.multi_face_landmarks

    if face_landmarks:
        # Get left eye landmark points from face
        left_eye = face_landmarks[0].landmark[145]
        # Get right eye landmark points from face
        right_eye = face_landmarks[0].landmark[374]
        # Get nose landmark points from face
        nose = face_landmarks[0].landmark[4]
        # Get chin landmark points from face
        chin = face_landmarks[0].landmark[152]
        # Get eye_midpoint landmark points from face
        eye_midpoint = face_landmarks[0].landmark[8]
        # Get lef_ear landmark points from face
        left_ear = face_landmarks[0].landmark[234]
        # Get right_ear landmark points from face
        right_ear = face_landmarks[0].landmark[454]
        # Get left_eye_top landmark points from face
        left_eye_top = face_landmarks[0].landmark[159]

        # Draw the nose landmark point on the frame
        nose_x = int(nose.x * frame.shape[1])
        nose_y = int(nose.y * frame.shape[0])
        cv2.circle(frame, (nose_x, nose_y), 5, (0, 255, 0), -1)

        # Draw the chin landmark point on the frame
        chin_x = int(chin.x * frame.shape[1])
        chin_y = int(chin.y * frame.shape[0])
        cv2.circle(frame, (chin_x, chin_y), 5, (0, 255, 0), -1)

        # Draw the eye_midpoint landmark point on the frame
        eye_midpoint_x = int(eye_midpoint.x * frame.shape[1])
        eye_midpoint_y = int(eye_midpoint.y * frame.shape[0])
        cv2.circle(frame, (eye_midpoint_x, eye_midpoint_y), 5, (0, 255, 0), -1)

        # Draw the left eye landmark point on the frame
        left_eye_x = int(left_eye.x * frame.shape[1])
        left_eye_y = int(left_eye.y * frame.shape[0])
        cv2.circle(frame, (left_eye_x, left_eye_y), 5, (0, 255, 0), -1)

        # Draw the left_eye_top landmark point on the frame
        left_eye_top_x = int(left_eye_top.x * frame.shape[1])
        left_eye_top_y = int(left_eye_top.y * frame.shape[0])
        cv2.circle(frame, (left_eye_top_x, left_eye_top_y), 5, (0, 0, 255), -1)


        # Draw the right eye landmark point on the frame
        right_eye_x = int(right_eye.x * frame.shape[1])
        right_eye_y = int(right_eye.y * frame.shape[0])
        cv2.circle(frame, (right_eye_x, right_eye_y), 5, (0, 255, 0), -1)

        # Draw the left ear landmark point on the frame
        left_ear_x = int(left_ear.x * frame.shape[1])
        left_ear_y = int(left_ear.y * frame.shape[0])
        cv2.circle(frame, (left_ear_x, left_ear_y), 5, (0, 255, 0), -1)

        # Draw the right ear landmark point on the frame
        right_ear_x = int(right_ear.x * frame.shape[1])
        right_ear_y = int(right_ear.y * frame.shape[0])
        cv2.circle(frame, (right_ear_x, right_ear_y), 5, (0, 255, 0), -1)

        # Draw a line from the nose landmark to eye_midpoint landmark
        cv2.line(frame, (chin_x, chin_y), (eye_midpoint_x, eye_midpoint_y), (255, 255, 0), 1)
        # Draw a line from right_ear to left_ear
        cv2.line(frame, (right_ear_x, right_ear_y), (left_ear_x, left_ear_y), (255, 255, 0), 1)
        # Draw a line from left_eye landmark to the right_eye landmark
        cv2.line(frame, (right_eye_x, right_eye_y), (left_eye_x, left_eye_y), (255, 255, 0), 1)


        # Processing the orientations
        # UP DOWN CONTROLS
        # Get left right ear landmark midpoint
        lr_mid_y = (left_ear_y + right_ear_y) // 2
        # Compare lr_mid_y with nose_y, if lr_mid_y is much greater than nose_y, then user is looking down
        if lr_mid_y - ud_treshold > nose_y:
            ud_control = "DOWN"
        elif lr_mid_y + ud_treshold < nose_y:
            ud_control = "UP"

        # LEFT RIGHT CONTROLS
        # Compare left_year_y with right_ear_y, if left_year_y is much greater than right_ear_y, go left, else right
        if left_ear_y > right_ear_y +lr_treshold:
            lr_control = "RIGHT"
        elif left_ear_y < right_ear_y - lr_treshold:
            lr_control = "LEFT"

        # ACTION CONTROL
        # Compare the y coords of left_eye_top and left_eye_bottom, if they are close, then press Space
        if abs(left_eye_top.y - left_eye.y) < 0.01:
            action_control = "SPACE"


        # Control the movement of the kart using pyautogui
        moves = [lr_control,ud_control]
        isKeyDown = False
        for move in moves:
            if move != 'CENTER':
                pyautogui.keyDown(move)
                isKeyDown = True

        if isKeyDown:
            pyautogui.sleep(0.1)
            for move in moves:
                if move != 'CENTER':
                    pyautogui.keyUp(move)

        # Control the action of the user when SPACE is pressed
        if action_control == "SPACE":
            pyautogui.keyDown('SPACE')
            pyautogui.sleep(0.1)
            pyautogui.keyUp('SPACE')





    # Write text on the frame to display the lr_control value
    cv2.putText(frame, 'LR: '+lr_control, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    # Write text on the frame to display the ud_control value
    cv2.putText(frame, 'UD: '+ud_control, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    # Write text on the frame to display the action_control value
    # cv2.putText(frame, 'AC: '+action_control, (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Get hand from webcam feed using hand_mesh
    hand = hand_mesh.process(rgb_frame)
    # Get hand landmark points from hand
    hand_landmarks = hand.multi_hand_landmarks

    # Display the webcam feed
    cv2.imshow('SmashVision', frame)
    cv2.setWindowProperty('SmashVision', cv2.WND_PROP_TOPMOST, 1)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
