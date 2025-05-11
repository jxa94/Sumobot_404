#ifndef BASIC_H
#define BASIC_H

#include <Arduino.h>

// BTS7960 Motor control pins
// Motor 1 (Left)
#define M1_RPWM 11  // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M1_LPWM 10  // Left/Backward PWM pin - connect to LPWM on BTS7960
// EN pins are connected to 5V directly

// Motor 2 (Right)
#define M2_RPWM 6   // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M2_LPWM 5   // Left/Backward PWM pin - connect to LPWM on BTS7960
// EN pins are connected to 5V directly

// Sensor Pin Definitions
#define BUMP_LEFT 8
#define BUMP_RIGHT 9
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Sensor Constants
extern const int NUM_IR_READINGS;
extern const int IR_MAX_DISTANCE;
extern const int IR_MIN_DISTANCE;
extern const int BUMP_LEFT_BIT;
extern const int BUMP_RIGHT_BIT;

// Global sensor state variable
extern int bumpSensorState;

// Function declarations for movement
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int leftSpeed, int rightSpeed);
void turnRight(int leftSpeed, int rightSpeed);
void stopMovement();

// Function declarations for sensors
int getIRDistance(int sensorPin);
long getUltrasonicDistance();
void updateSensorStates();

#endif // BASIC_H