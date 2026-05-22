import speech_recognition as sr
import serial
import time



ser = serial.Serial('COM7', 115200)
time.sleep(2)

recognizer = sr.Recognizer()

print("Voice control started...")

while True:
    with sr.Microphone() as source:
        print("Say command...")

        try:
            audio = recognizer.listen(source, timeout=5)

            text = recognizer.recognize_google(audio).lower()
            # text = recognizer.recognize_google(audio, language="ar-EG").lower()
            print("You said:", text)


            if "forward" in text:
                ser.write(b'h\n')
                print("Move Forward")


            elif "back" in text:
                ser.write(b'b\n')
                print("Move Backward")

            elif "right" in text:
                ser.write(b'r\n')
                print("Turn Right")

            elif "left" in text:
                ser.write(b'l\n')
                print("Turn Left")

            elif "shoot" in text:
                ser.write(b's\n')
                print("Kick")

            elif "hand" in text:
                ser.write(b'p\n')
                print("Move With Hand")

            # INITIAL POSITION
            elif "stand" in text:
                ser.write(b'i\n')
                print("Initial Position")

        except sr.UnknownValueError:
            print("Didn't understand")

        except Exception as e:
            print("Error:", e)