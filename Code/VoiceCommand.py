import queue
import sounddevice as sd
import json
from vosk import Model, KaldiRecognizer
import pyttsx3
import random
import time

# ---------------------------
# Initialize Text-to-Speech
# ---------------------------
engine = pyttsx3.init()
engine.setProperty('rate', 150)  # speaking speed
engine.setProperty('volume', 1.0)  # volume level

voices = engine.getProperty('voices')
engine.setProperty('voice', voices[0].id)  # select a voice you like (male/female)

def speak(text):
    engine.say(text)
    engine.runAndWait()

# ---------------------------
# Load Vosk Model
# ---------------------------
model_path = r"C:\Users\Elyas\OneDrive - The University of Colorado Denver\Desktop\Projects\robotic-arm\Code\vosk-model-small-en-us-0.15"  # <<< put your correct path here
model = Model(model_path)

# ---------------------------
# Setup Audio Stream
# ---------------------------
q = queue.Queue()

def audio_callback(indata, frames, time, status):
    if status:
        print(status)
    q.put(bytes(indata))

samplerate = 16000  # Vosk expects 16kHz audio

stream = sd.RawInputStream(samplerate=samplerate, blocksize=8000, device=None,
                           dtype='int16', channels=1, callback=audio_callback)

recognizer = KaldiRecognizer(model, samplerate)



def speak(text, emotion="neutral"):
    if emotion == "excited":
        engine.setProperty('rate', 180)
    elif emotion == "calm":
        engine.setProperty('rate', 130)
    else:
        engine.setProperty('rate', 150)

    time.sleep(random.uniform(0.1, 0.4))  # Small thinking pause
    engine.say(text)
    engine.runAndWait()

def handle_command(text):
    text = text.lower()
    print(f"Recognized Command: {text}")

    if "pick up" in text:
        phrase = random.choice([
            "On it boss, getting the object!",
            "Phew, hope this one's light!",
            "Time to grab the prize!"
        ])
        speak(phrase, emotion="excited")
        # Robotic arm pickup code

    elif "drop" in text or "release" in text:
        phrase = random.choice([
            "Releasing... hope it doesn't break!",
            "Here you go, nice and easy.",
            "Dropping it like it's hot!"
        ])
        speak(phrase, emotion="calm")
        # Robotic arm drop code

    elif "move to bin" or "move to ben" in text:
        phrase = random.choice([
            "Marching to the bin!",
            "On a mission to the bin!",
            "Navigating to target destination!"
        ])
        speak(phrase, emotion="excited")
        # Robotic arm move code

    elif "shutdown" in text or "stop" in text:
        speak("Going to sleep mode. See you soon!", emotion="calm")
        exit(0)

    else:
        speak("Huh? Can you repeat that?", emotion="neutral")
# ---------------------------
# Main Loop
# ---------------------------
def main():
    print("🎙️ Voice command system ready. Speak a command!")

    speak("System online and ready for your commands!")

    with stream:
        while True:
            data = q.get()
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "")
                if text:
                    handle_command(text)

if __name__ == "__main__":
    main()