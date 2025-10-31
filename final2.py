# final_controller_v3.py
# This version includes the serial fix AND re-adds visual hand recognition feedback.

import cv2
import mediapipe as mp
import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
WEBCAM_INDEX = 0

# --- SETUP ---
print("Starting AI Ghost Hand Controller V3 (with visual feedback)...")

# 1. Setup Serial Connection to Arduino
try:
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1, dsrdtr=False)
    print(f"Successfully connected to Arduino on {SERIAL_PORT}")
    time.sleep(2)  # Wait for the Arduino to be ready
except Exception as e:
    print(f"Error: Could not connect to Arduino on {SERIAL_PORT}. Please check the port and permissions.")
    print(e)
    exit()

# 2. Setup MediaPipe and OpenCV
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils  # This was missed in V2
cap = cv2.VideoCapture(WEBCAM_INDEX)

print("Setup complete. Looking for gestures...")

# --- MAIN LOOP ---
last_command = ""
command_cooldown = 2
last_command_time = 0

while cap.isOpened():
    success, image = cap.read()
    if not success:
        continue

    image = cv2.flip(image, 1)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_image)

    current_command = ""

    # --- Drawing and Gesture detection logic ---
    if results.multi_hand_landmarks:
        # Draw landmarks FIRST so they are visible
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)  # Re-added this

        hand_landmarks = results.multi_hand_landmarks[0]
        try:
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            index_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
            middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            middle_pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
            ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
            ring_pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
            pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
            pinky_pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]

            is_fist = (
                        index_tip.y > index_pip.y and middle_tip.y > middle_pip.y and ring_tip.y > ring_pip.y and pinky_tip.y > pinky_pip.y)
            is_open_palm = (
                        index_tip.y < index_pip.y and middle_tip.y < middle_pip.y and ring_tip.y < ring_pip.y and pinky_tip.y < pinky_pip.y)

            if is_fist:
                current_command = "FIST"
                cv2.putText(image, "FIST", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)  # Re-added this
            elif is_open_palm:
                current_command = "OPEN_PALM"
                cv2.putText(image, "OPEN_PALM", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)  # Re-added this

        except Exception as e:
            pass

    # --- Send Command to Arduino ---
    if current_command and current_command != last_command and (time.time() - last_command_time) > command_cooldown:
        print(f"Gesture Detected: {current_command}. Sending command to Arduino.")
        ser.write(f"{current_command}\n".encode('utf-8'))

        last_command = current_command
        last_command_time = time.time()

    cv2.imshow('AI Ghost Hand - Controller V3', image)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# --- CLEANUP ---
print("Shutting down...")
cap.release()
cv2.destroyAllWindows()
ser.close()
print("Done.")