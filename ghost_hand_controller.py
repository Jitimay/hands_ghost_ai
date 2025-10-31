# final_controller.py
# This is the final, corrected Python script for AI Ghost Hands.

import cv2
import mediapipe as mp
import serial
import time

# --- CONFIGURATION ---
# IMPORTANT: Make sure this serial port is correct for your system.
SERIAL_PORT = '/dev/ttyACM0'  # For Linux. For Windows, it might be 'COM3', 'COM4', etc.
BAUD_RATE = 115200  # Must match the Arduino's Serial.begin() rate.
WEBCAM_INDEX = 0

# --- SETUP ---
print("Starting AI Ghost Hand Controller...")

# 1. Setup Serial Connection to Arduino
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Successfully connected to Arduino on {SERIAL_PORT}")
    time.sleep(2)  # Wait for the connection to establish
except Exception as e:
    print(f"Error: Could not connect to Arduino on {SERIAL_PORT}. Please check the port and permissions.")
    print(e)
    exit()

# 2. Setup MediaPipe Hand Tracking
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# 3. Setup OpenCV Video Capture
cap = cv2.VideoCapture(WEBCAM_INDEX)
if not cap.isOpened():
    print(f"Error: Could not open webcam with index {WEBCAM_INDEX}.")
    exit()

print("Setup complete. Looking for gestures...")

# --- MAIN LOOP ---
last_command = ""
command_cooldown = 2  # 2 seconds between commands
last_command_time = 0

while cap.isOpened():
    success, image = cap.read()
    if not success:
        continue

    # Flip the image horizontally for a selfie-view display
    image = cv2.flip(image, 1)
    # Convert the BGR image to RGB for MediaPipe.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Process the image and find hands
    results = hands.process(rgb_image)

    current_command = ""

    # Gesture detection logic
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]

        # Draw landmarks on the image for visualization
        mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # --- GESTURE LOGIC ---
        try:
            # Get coordinates for finger tips and PIP joints (second joint from the tip)
            thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            index_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
            middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            middle_pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
            ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
            ring_pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
            pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
            pinky_pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]

            # Simple check for FIST: all finger tips are below their PIP joint.
            # The y-coordinate is lower for higher points on the screen.
            is_fist = (index_tip.y > index_pip.y and
                       middle_tip.y > middle_pip.y and
                       ring_tip.y > ring_pip.y and
                       pinky_tip.y > pinky_pip.y)

            # Simple check for OPEN_PALM: all finger tips are above their PIP joint.
            is_open_palm = (index_tip.y < index_pip.y and
                            middle_tip.y < middle_pip.y and
                            ring_tip.y < ring_pip.y and
                            pinky_tip.y < pinky_pip.y)

            if is_fist:
                current_command = "FIST"
                cv2.putText(image, "FIST", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif is_open_palm:
                current_command = "OPEN_PALM"
                cv2.putText(image, "OPEN_PALM", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        except Exception as e:
            print(f"Could not process landmarks: {e}")

    # --- Send Command to Arduino ---
    # Check if a new command is detected and if the cooldown has passed
    if current_command and current_command != last_command and (time.time() - last_command_time) > command_cooldown:
        print(f"Gesture Detected: {current_command}. Sending command to Arduino.")

        # Send the command as a byte string, with a newline character at the end.
        # The Arduino's readStringUntil('\n') depends on this newline.
        ser.write(f"{current_command}\n".encode('utf-8'))

        last_command = current_command
        last_command_time = time.time()

    # Display the resulting frame
    cv2.imshow('AI Ghost Hand - Controller', image)

    # Exit on 'q' key press
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# --- CLEANUP ---
print("Shutting down...")
cap.release()
cv2.destroyAllWindows()
ser.close()
print("Done.")