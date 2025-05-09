#ifndef MOVEMENT_H
#define MOVEMENT_H

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

// Function declarations for movement
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int leftSpeed, int rightSpeed);
void turnRight(int leftSpeed, int rightSpeed);
void stopMovement();

#endif // MOVEMENT_H