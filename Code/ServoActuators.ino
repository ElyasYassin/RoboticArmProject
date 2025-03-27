#include <Servo.h>

Servo servo1, servo2;

void setup() {
    Serial.begin(9600);
    servo1.attach(9);
    servo2.attach(10);
}

void loop() {
    if (Serial.available()) {
        int theta1 = Serial.parseInt();
        int theta2 = Serial.parseInt();
        
        servo1.write(theta1);
        servo2.write(theta2);
    }
}