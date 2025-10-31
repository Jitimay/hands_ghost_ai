# final_controller_v10_debug.py
# This version adds debug prints to diagnose why commands are not being sent.

import cv2
import mediapipe as mp
import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
WEBCAM_INDEX = 0

# --- SETUP ---
print("DEBUG: Starting AI Ghost Hand Controller V10 (Command Debug)...")

# 1. Setup Serial Connection to Arduino
try:
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1, dsrdtr=False)
    print(f"DEBUG: Successfully connected to Arduino on {SERIAL_PORT}")
    time.sleep(2)  # Wait for the Arduino to be ready
except Exception as e:
    print(f"ERROR: Could not connect to Arduino on {SERIAL_PORT}. Please check the port and permissions.")
    print(e)
    exit()

# 2. Setup MediaPipe and OpenCV
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils
cap = cv2.VideoCapture(WEBCAM_INDEX)

if not cap.isOpened():
    print(f"ERROR: Could not open webcam with index {WEBCAM_INDEX}. Exiting.")
    exit()
else:
    print("DEBUG: Webcam opened successfully.")

print("DEBUG: Setup complete. Looking for gestures...")
print("DEBUG: Entering main loop.")

# --- MAIN LOOP ---
last_command_left = ""
last_command_right = ""
command_cooldown = 1
last_command_time_left = 0
last_command_time_right = 0

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("DEBUG: Failed to read frame from webcam. Continuing...")
        continue

    image = cv2.flip(image, 1)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_image)

    current_commands = {"Left": "", "Right": ""}

    if results.multi_hand_landmarks:
        # print("DEBUG: Hand landmarks detected.") # Already confirmed by user
        for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
            handedness_label = results.multi_handedness[hand_idx].classification[0].label
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            try:
                thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                index_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
                middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                middle_pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
                ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
                ring_pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
                pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
                pinky_pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]

                is_fist = (index_tip.y > index_pip.y and middle_tip.y > middle_pip.y and ring_tip.y > ring_pip.y and pinky_tip.y > pinky_pip.y)
                is_open_palm = (index_tip.y < index_pip.y and middle_tip.y < middle_pip.y and ring_tip.y < ring_pip.y and pinky_tip.y < pinky_pip.y)
                is_thumbs_up = (thumb_tip.y < thumb_ip.y and index_tip.y > index_pip.y and middle_tip.y > middle_pip.y and ring_tip.y > ring_pip.y and pinky_tip.y > pinky_pip.y)

                if is_fist:
                    current_commands[handedness_label] = "FIST"
                    cv2.putText(image, f"{handedness_label}: FIST", (50, 50 + hand_idx*40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                elif is_open_palm:
                    current_commands[handedness_label] = "OPEN_PALM"
                    cv2.putText(image, f"{handedness_label}: OPEN_PALM", (50, 50 + hand_idx*40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                elif is_thumbs_up:
                    current_commands[handedness_label] = "THUMBS_UP"
                    cv2.putText(image, f"{handedness_label}: THUMBS_UP", (50, 50 + hand_idx*40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            except Exception as e:
                print(f"ERROR: Gesture detection failed: {e}")
                pass
    else:
        # print("DEBUG: No hand landmarks detected.") # Not needed if user sees gestures
        pass

    # --- Send Commands to Arduino ---
    for hand_label, cmd in current_commands.items():
        print(f"DEBUG: Processing hand_label={hand_label}, cmd={cmd}")
        if hand_label == "Left":
            # Left Hand controls Lamp
            if cmd and cmd != last_command_left and (time.time() - last_command_time_left) > command_cooldown:
                print(f"DEBUG: Left hand conditions met. cmd={cmd}, last_cmd={last_command_left}")
                if cmd == "OPEN_PALM" or cmd == "FIST":
                    full_cmd = f"LEFT_{cmd}"
                    print(f"DEBUG: Gesture Detected: {full_cmd}. Sending command to Arduino.")
                    ser.write(f"{full_cmd}\n".encode('utf-8'))
                    last_command_left = cmd
                    last_command_time_left = time.time()
                else:
                    print(f"DEBUG: Left hand cmd '{cmd}' not 'OPEN_PALM' or 'FIST'. Not sending.")
            else:
                print(f"DEBUG: Left hand conditions NOT met. cmd={cmd}, last_cmd={last_command_left}, cooldown={(time.time() - last_command_time_left)}")
        elif hand_label == "Right":
            # Right Hand controls Pump
            if cmd and cmd != last_command_right and (time.time() - last_command_time_right) > command_cooldown:
                print(f"DEBUG: Right hand conditions met. cmd={cmd}, last_cmd={last_command_right}")
                if cmd == "THUMBS_UP":
                    full_cmd = f"RIGHT_{cmd}"
                    print(f"DEBUG: Gesture Detected: {full_cmd}. Sending command to Arduino.")
                    ser.write(f"{full_cmd}\n".encode('utf-8'))
                    last_command_right = cmd
                    last_command_time_right = time.time()
                else:
                    print(f"DEBUG: Right hand cmd '{cmd}' not 'THUMBS_UP'. Not sending.")
            else:
                print(f"DEBUG: Right hand conditions NOT met. cmd={cmd}, last_cmd={last_command_right}, cooldown={(time.time() - last_command_time_right)}")

    cv2.imshow('AI Ghost Hand - Controller V10 (Command Debug)', image)
    # print("DEBUG: Image displayed.") # Too much output

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# --- CLEANUP ---
print("DEBUG: Shutting down...")
cap.release()
cv2.destroyAllWindows()
ser.close()
print("DEBUG: Done.")