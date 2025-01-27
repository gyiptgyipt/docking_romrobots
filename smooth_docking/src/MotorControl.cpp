#include "MotorControl.hpp"

MotorControl::MotorControl(int in1, int in2, int in3, int in4, int en1, int en2) {
    IN1 = in1;
    IN2 = in2;
    IN3 = in3;
    IN4 = in4;
    EN1 = en1;
    EN2 = en2;
}

void MotorControl::setupMotors() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
}

void MotorControl::moveForward(int speed) {
    analogWrite(EN1, speed);
    analogWrite(EN2, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void MotorControl::moveBackward(int speed) {
    analogWrite(EN1, speed);
    analogWrite(EN2, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void MotorControl::moveLeft(int speed) {
    analogWrite(EN1, speed);
    analogWrite(EN2, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void MotorControl::moveRight(int speed) {
    analogWrite(EN1, speed);
    analogWrite(EN2, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void MotorControl::stopMotors() {
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
