# Robotic Arm: Autonomous Pick and Place System

## 📚 Project Overview

This project implements a **fully autonomous robotic arm** capable of:

- Detecting an object's **color** (purple or green) placed on its gripper
- Identifying the correct **bin location** using **Aruco markers** and **computer vision**
- Solving **inverse kinematics (IK)** to calculate joint angles for precise movement
- Moving towards the bin and **placing the object**
- Providing **audio feedback** using **text-to-speech (TTS)**
- Recovering from errors gracefully if detections or movements fail

The full system integrates **computer vision**, **robotics control**, **voice feedback**, and **real-time error handling** for a seamless demonstration.

---

## 🛠 Project Structure

| Folder/File | Purpose |
|:------------|:--------|
| `InverseKinematics.py` | Solve inverse kinematics and optimize final angles |
| `ArduinoComms.py` | Communicate with Arduino via serial commands |
| `VoiceCommand.py` | Text-to-speech functionality for speaking updates |
| `CVCode.py` | Detect Aruco markers, calculate world coordinates |
| `ColorDetection.py` | Identify object color from gripper camera |
| `Script.py` | Main script that ties everything together (central control loop) |
| `Arduino Code (not shown)` | Receives angles and moves servos (base, shoulder, elbow, gripper) |

---

## 🧠 How It Works

1. **Startup**
   - Robot says: "System online and ready."
   
2. **Object Detection**
   - Waits for an object to be placed on the gripper.
   - Uses a camera to **detect the object's color** (purple or green).

3. **Bin Identification**
   - Launches **ArUco marker detection** to find bins.
   - **Stabilizes coordinates** by averaging multiple detections.

4. **Movement Planning**
   - Solves **inverse kinematics** based on detected bin location.
   - Smoothly moves towards the target bin using optimized servo angles.

5. **Object Placement**
   - Opens the gripper to drop the object into the bin.
   - Returns to a **neutral standby position** afterward.

6. **Voice Feedback**
   - Throughout the process, the robot **speaks** status updates such as:
     - "Waiting for object."
     - "Purple object detected."
     - "Moving to bin."
     - "Object placed."

---

## 🖥️ Hardware Requirements

- Robotic arm (3DoF minimum: base, shoulder, elbow + gripper)
- **Servo motors** (capable of 0–180°)
- **Arduino** for low-level motor control
- **Raspberry Pi** or Laptop (Python environment)
- **Webcam** (for color and marker detection)
- External 5V/6V **power supply** for servos (recommended)

For more details check out the stl files for the model, credit goes to https://www.youtube.com/watch?v=wnse-NYCXL4 for the robotic arm and https://www.thingiverse.com/thing:1015238 for the gripper.
---

## 🧩 Software Stack

- Python 3.8+
- OpenCV (`cv2`) — for computer vision and ArUco marker tracking
- PySerial — for Arduino communication
- Vosk — for voice (TTS)
- NumPy — for math operations

Install dependencies:

```bash
pip install -r requirements.txt

IMPORTANT!
- Change the path for the model in the voicecommand.py file to your own path
