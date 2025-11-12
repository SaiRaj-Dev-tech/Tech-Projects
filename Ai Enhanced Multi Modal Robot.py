import cv2
import mediapipe as mp
import pyfirmata2
import pyttsx3
import queue
import threading
import speech_recognition as sr

# ========== Speech Engine ==========
engine = pyttsx3.init()
engine.setProperty("rate", 150)
engine.setProperty("volume", 1)
speech_queue = queue.Queue()

def process_speech():
    while True:
        text = speech_queue.get()
        if text == "EXIT":
            break
        engine.say(text)
        engine.runAndWait()

speech_thread = threading.Thread(target=process_speech, daemon=True)
speech_thread.start()

# ========== Arduino Setup ==========
port = 'COM10'  # Change to your Arduino port
board = pyfirmata2.Arduino(port)
print("SUCCESS: Arduino connected")

# Motor A (Left)
in1_pin = 8
in2_pin = 9
ena_pin = 10

# Motor B (Right)
in3_pin = 11
in4_pin = 12
enb_pin = 5

# Set pin modes
for pin in [in1_pin, in2_pin, in3_pin, in4_pin]:
    board.digital[pin].mode = pyfirmata2.OUTPUT

board.digital[ena_pin].mode = pyfirmata2.PWM
board.digital[enb_pin].mode = pyfirmata2.PWM

# Default speed
default_speed = 0.1
board.digital[ena_pin].write(default_speed)
board.digital[enb_pin].write(default_speed)
print(f"Motor speed set to {int(default_speed * 100)}%")

motor_state = "stopped"

# ========== Motor Control Functions ==========
def motor_forward():
    global motor_state
    if motor_state != "forward":
        board.digital[in1_pin].write(1)
        board.digital[in2_pin].write(0)
        board.digital[ena_pin].write(default_speed)

        board.digital[in3_pin].write(1)
        board.digital[in4_pin].write(0)
        board.digital[enb_pin].write(default_speed)

        motor_state = "forward"
        print(" Motors running forward")
        speech_queue.put("Motors running forward")

def motor_backward():
    global motor_state
    if motor_state != "backward":
        board.digital[in1_pin].write(0)
        board.digital[in2_pin].write(1)
        board.digital[ena_pin].write(default_speed)

        board.digital[in3_pin].write(0)
        board.digital[in4_pin].write(1)
        board.digital[enb_pin].write(default_speed)

        motor_state = "backward"
        print("Motors running backward")
        speech_queue.put("Motors running backward")

def motor_stop():
    global motor_state
    if motor_state != "stopped":
        board.digital[in1_pin].write(0)
        board.digital[in2_pin].write(0)
        board.digital[ena_pin].write(0)

        board.digital[in3_pin].write(0)
        board.digital[in4_pin].write(0)
        board.digital[enb_pin].write(0)

        motor_state = "stopped"
        print("Motors stopped")
        speech_queue.put("Motors stopped")

# ========== Text Command Mode ==========
def text_command_mode():
    print("Text Command Mode Activated!")
    while True:
        cmd = input("Enter 'f'=forward, 'b'=backward, 's'=stop, 'e'=exit: ").strip().lower()
        if cmd == 'f':
            motor_forward()
        elif cmd == 'b':
            motor_backward()
        elif cmd == 's':
            motor_stop()
        elif cmd == 'e':
            motor_stop()
            print("Exiting text mode...")
            break
        else:
            print("Invalid command. Try again.")

# ========== Gesture Control Mode ==========
def gesture_control_mode():
    print("Gesture Control Mode Activated!")
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
    mp_drawing = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera not detected")
        return

    print("Index = Forward | Middle = Backward | Ring = Stop | 'q' to Quit")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            fingers_up = [False] * 5

            if results.multi_hand_landmarks:
                for hand in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
                    tips = [4, 8, 12, 16, 20]
                    lowers = [3, 7, 11, 15, 19]
                    for i in range(5):
                        tip_y = hand.landmark[tips[i]].y * h
                        lower_y = hand.landmark[lowers[i]].y * h
                        fingers_up[i] = tip_y < lower_y

            if fingers_up[1]:  # Index
                motor_forward()
            elif fingers_up[2]:  # Middle
                motor_backward()
            elif fingers_up[3]:  # Ring
                motor_stop()

            cv2.putText(frame, "Index=Forward | Middle=Backward | Ring=Stop | 'q'=Quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.imshow("Gesture Motor Control", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                motor_stop()
                break
    except Exception as e:
        print(f" Error: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        hands.close()

# ========== Voice Command Mode ==========
def voice_command_mode():
    print(" Voice Command Mode Activated!")
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("Say commands: 'go forward', 'go backward', 'stop', 'exit'")
    with mic as source:
        recognizer.adjust_for_ambient_noise(source)

    try:
        while True:
            with mic as source:
                print("Listening...")
                audio = recognizer.listen(source)

            try:
                command = recognizer.recognize_google(audio).lower()
                print(f"You said: {command}")

                if "forward" in command:
                    motor_forward()
                elif "backward" in command:
                    motor_backward()
                elif "stop" in command:
                    motor_stop()
                elif "exit" in command:
                    motor_stop()
                    print("Exiting voice command mode...")
                    break
                else:
                    print("Unrecognized command")

            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError:
                print("Speech recognition service error")

    except KeyboardInterrupt:
        print(" Voice mode manually interrupted.")

# ========== Main ==========
def main():
    try:
        print("Select Control Mode:")
        print("1. Text Command Mode")
        print("2. Gesture Control Mode")
        print("3. Voice Command Mode")
        choice = input("Enter choice (1, 2 or 3): ").strip()

        if choice == '1':
            text_command_mode()
        elif choice == '2':
            gesture_control_mode()
        elif choice == '3':
            voice_command_mode()
        else:
            print("Invalid choice. Exiting...")

    finally:
        motor_stop()
        speech_queue.put("EXIT")
        speech_thread.join(timeout=3)
        board.exit()
        print("Clean exit completed.")

if __name__ == "__main__":
    main()
