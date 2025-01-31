#include <BluetoothSerial.h>
#include "MotorControl.hpp"

// Pin assignments
#define IN1 D9
#define IN2 D8
#define IN3 D13
#define IN4 D12
#define EN1 D10
#define EN2 D11

// Initialize motor control object
MotorControl motor(IN1, IN2, IN3, IN4, EN1, EN2);
bool right = 0;
bool left = 0;
bool mid = 0;
// Bluetooth Serial
BluetoothSerial SerialBT;
void charge();

const int irSensorPin = A0;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("MotorController"); // Bluetooth device name
    motor.setupMotors();
    Serial.println("Bluetooth Device is Ready to Pair");

}
void charge(){
    while(1){

     if (analogRead(irSensorPin) >= 0 && analogRead(irSensorPin) < 300 ){
        motor.moveBackward(80);
        right = 0;
        left = 0;
        delay(100);

    }
    if (analogRead(irSensorPin) >= 300 && analogRead(irSensorPin) <= 1500 && right == 0 && left == 0){
        motor.moveRight(80);
        delay(100);
        if(analogRead(irSensorPin) > 1500){
            motor.moveLeft(80);
            right = 1;
        }
    }
    if (analogRead(irSensorPin) >= 300 && analogRead(irSensorPin) <= 1500 && right == 1 && left == 0){
        motor.moveLeft(80);
        left = 1;
        delay(100);
    }
    if(analogRead(irSensorPin) > 1500 && right == 1 && left == 1){
            motor.stopMotors();
            right = 0;
            left = 0;
            delay(100);
    }
    }
}

void loop() {
    while(SerialBT.connected(1) == 0);
    int irSensorValue = analogRead(irSensorPin);

    // Print sensor values for debugging
    SerialBT.print("IR Sensor : ");
    SerialBT.println(irSensorValue);
    
    if (SerialBT.available()) {
        char command = SerialBT.read();

        switch (command) {
            case 'F':
                motor.moveForward(255);
                SerialBT.println("Moving forward at full speed.");
                break;
            case 'f':
                motor.moveForward(127);
                SerialBT.println("Moving forward at half speed.");
                break;
            case 'B':
                motor.moveBackward(255);
                SerialBT.println("Moving backward at full speed.");
                break;
            case 'b':
                motor.moveBackward(127);
                SerialBT.println("Moving backward at half speed.");
                break;
            case 'C':
                charge();
                SerialBT.println("Charging.");
                break;
            case 'L':
                motor.moveLeft(255);
                SerialBT.println("Turning left at full speed.");
                break;
            case 'l':
                motor.moveLeft(127);
                SerialBT.println("Turning left at half speed.");
                break;
            case 'R':
                motor.moveRight(255);
                SerialBT.println("Turning right at full speed.");
                break;
            case 'r':
                motor.moveRight(127);
                SerialBT.println("Turning right at half speed.");
                break;
            case 'S':
            case 's':
                motor.stopMotors();
                SerialBT.println("Motors stopped.");
                break;
            default:
                SerialBT.println("Invalid command. Use F, f, B, b, L, l, R, r, S, or s.");
                break;
        }
    }
        
}

