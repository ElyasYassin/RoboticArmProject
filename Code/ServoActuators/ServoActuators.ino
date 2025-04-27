#include <Servo.h>

Servo baseServo, lowerServo, upperServo, gripperServo;

const int BASE_PIN = 3, LOWER_PIN = 9, UPPER_PIN = 6, GRIPPER_PIN = 10;

int baseTarget = 90, lowerTarget = 90, upperTarget = 90, gripperTarget = 90;
int baseAngle = 90, lowerAngle = 90, upperAngle = 90, gripperAngle = 90;

unsigned long lastUpdate = 0;
const int updateInterval = 23; // ms between position steps

// Movement limits
const int BASE_MIN = 0, BASE_MAX = 180;
const int LOWER_MIN = 50, LOWER_MAX = 155;
const int UPPER_MIN = 0, UPPER_MAX = 180;
const int GRIPPER_MIN = 0, GRIPPER_MAX = 180;

String inputString = "";
bool inputComplete = false;

void setup() {
  Serial.begin(9600);

  baseServo.attach(BASE_PIN);
  lowerServo.attach(LOWER_PIN);
  upperServo.attach(UPPER_PIN);
  gripperServo.attach(GRIPPER_PIN);

  baseServo.write(baseAngle);
  lowerServo.write(lowerAngle);
  upperServo.write(upperAngle);
  gripperServo.write(gripperAngle);

  inputString.reserve(50);
}

void loop() {
  readSerialCommand();
  updateServosNonBlocking();
}

// Reads serial commands like "A:100;B:120;C:90;G:30"
void readSerialCommand() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (inputComplete) {
    int b = extractValue(inputString, "A:");
    int l = extractValue(inputString, "B:");
    int u = extractValue(inputString, "C:");
    int g = extractValue(inputString, "G:");

    if (b != -1) baseTarget = constrain(b, BASE_MIN, BASE_MAX);
    if (l != -1) lowerTarget = constrain(l, LOWER_MIN, LOWER_MAX);
    if (u != -1) upperTarget = constrain(u, UPPER_MIN, UPPER_MAX);
    if (g != -1) gripperTarget = constrain(g, GRIPPER_MIN, GRIPPER_MAX);

    inputString = "";
    inputComplete = false;
  }
}

// Moves each servo step-by-step without blocking
void updateServosNonBlocking() {
  unsigned long now = millis();
  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;

    moveServoStep(baseServo, baseAngle, baseTarget);
    moveServoStep(lowerServo, lowerAngle, lowerTarget);
    moveServoStep(upperServo, upperAngle, upperTarget);
    moveServoStep(gripperServo, gripperAngle, gripperTarget);
  }
}

void moveServoStep(Servo& servo, int& currentAngle, int targetAngle) {
  if (currentAngle < targetAngle) currentAngle++;
  else if (currentAngle > targetAngle) currentAngle--;

  servo.write(currentAngle);
}

// Extracts angle from "TAG:value"
int extractValue(const String& cmd, const String& tag) {
  int start = cmd.indexOf(tag);
  if (start == -1) return -1;
  int end = cmd.indexOf(';', start);
  if (end == -1) end = cmd.length();
  return cmd.substring(start + 2, end).toInt();
}
