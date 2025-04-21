#include <Servo.h>

// Servo Objects
Servo baseServo;
Servo lowerServo;
Servo upperServo;
Servo gripperServo;

// Servo pins
const int BASE_PIN = 3;
const int LOWER_PIN = 9;
const int UPPER_PIN = 6;
const int GRIPPER_PIN = 10;

// Movement limits
const int BASE_MIN = 60, BASE_MAX = 120;
const int LOWER_MIN = 45, LOWER_MAX = 135;
const int UPPER_MIN = 60, UPPER_MAX = 120;
const int GRIPPER_MIN = 0, GRIPPER_MAX = 180;

// Delay between small steps (ms)
const int STEP_DELAY = 150;

// Incoming serial buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600); // Start Serial comms with Raspberry Pi

  baseServo.attach(BASE_PIN);
  lowerServo.attach(LOWER_PIN);
  upperServo.attach(UPPER_PIN);
  gripperServo.attach(GRIPPER_PIN);

  inputString.reserve(50); // Allocate memory for buffer
}

void loop() {
  if (stringComplete) {
    parseAndMove(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void parseAndMove(String command) {
  int base = -1, lower = -1, upper = -1, gripper = -1;

  int aIndex = command.indexOf("A:");
  int bIndex = command.indexOf("B:");
  int cIndex = command.indexOf("C:");
  int gIndex = command.indexOf("G:");

  if (aIndex != -1) base = command.substring(aIndex + 2, command.indexOf(';', aIndex)).toInt();
  if (bIndex != -1) lower = command.substring(bIndex + 2, command.indexOf(';', bIndex)).toInt();
  if (cIndex != -1) upper = command.substring(cIndex + 2, command.indexOf(';', cIndex)).toInt();
  if (gIndex != -1) gripper = command.substring(gIndex + 2).toInt(); // Last one doesn't need ;

  if (base != -1) baseServo.write(constrain(base, BASE_MIN, BASE_MAX));
  if (lower != -1) lowerServo.write(constrain(lower, LOWER_MIN, LOWER_MAX));
  if (upper != -1) upperServo.write(constrain(upper, UPPER_MIN, UPPER_MAX));
  if (gripper != -1) gripperServo.write(constrain(gripper, GRIPPER_MIN, GRIPPER_MAX));

  delay(STEP_DELAY);
}