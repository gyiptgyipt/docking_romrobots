#pragma once
#include <Arduino.h>

class MotorControl {
    public:
        MotorControl(int in1, int in2, int in3, int in4, int en1, int en2);
        void setupMotors();
        void moveForward(int speed);
        void moveBackward(int speed);
        void moveRight(int speed);
        void moveLeft(int speed);
        void stopMotors();
    private:
        int IN1, IN2, IN3, IN4, EN1, EN2;
};