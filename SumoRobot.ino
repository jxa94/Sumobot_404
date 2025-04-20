#include <Arduino.h>

// Motor pins: Digital output
#define M1L 11
#define M1R 10
#define M2L 5
#define M2R 6

// Starter switch: Digital input
#define JSUMO_SWITCH 4

// Line sensors: Digital input
#define LINE_SENSOR_FL 2  // Front-left
#define LINE_SENSOR_FR 3  // Front-right
#define LINE_SENSOR_BL 12 // Back-left
#define LINE_SENSOR_BR 13 // Back-right

// Bump sensors
#define BUMP_LEFT 7
#define BUMP_RIGHT 8

// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int ONstate = 1;
const int FRONT_BIT = 0;
const int LEFT_BIT = 1;
const int RIGHT_BIT = 2;
const int LINE_FL_BIT = 3;
const int LINE_FR_BIT = 4;
const int LINE_BL_BIT = 5;
const int LINE_BR_BIT = 6;
//I AM TESTING THE EDDIT
// Global variables
unsigned long exploreStartTime = 0;
unsigned long exploreDuration = 0;
unsigned long actionStartTime = 0;
unsigned long actionDuration = 200;
bool cond = false;
int sensorStates = 0;
int wheelSpeed = 200;
unsigned long waitStartTime = 0;
bool waitOnce = false;

// Movement Functions
void moveForward(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2L, LOW);
}

void turnRight(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2R, LOW);
}

void stopMovement() {
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
}

// Snake Movement
void snakeMovement() {
  static bool direction = true;
  static unsigned long lastChangeTime = millis();
  unsigned long currentTime = millis();

  if (currentTime - lastChangeTime >= 500) {
    direction = !direction;
    lastChangeTime = currentTime;
  }

  if (direction) {
    moveForward(255, 150);
  } else {
    moveForward(150, 255);
  }

  actionStartTime = millis();
  while (millis() - actionStartTime < 950) {
    if (!digitalRead(LINE_SENSOR_FR) || !digitalRead(LINE_SENSOR_FL) ||
        !digitalRead(LINE_SENSOR_BR) || !digitalRead(LINE_SENSOR_BL)) {
      break;
    }
  }
}

// Sensor State Update
void updateSensorStates() {
  sensorStates = (digitalRead(BUMP_RIGHT) << RIGHT_BIT) |
                 (digitalRead(BUMP_LEFT) << LEFT_BIT) |
                 (digitalRead(ULTRASONIC_ECHO) << FRONT_BIT) |
                 (!digitalRead(LINE_SENSOR_FL) << LINE_FL_BIT) |
                 (!digitalRead(LINE_SENSOR_FR) << LINE_FR_BIT) |
                 (!digitalRead(LINE_SENSOR_BL) << LINE_BL_BIT) |
                 (!digitalRead(LINE_SENSOR_BR) << LINE_BR_BIT);
}

// Setup
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(M2R, OUTPUT);
  pinMode(M2L, OUTPUT);
  pinMode(M1L, OUTPUT);
  pinMode(M1R, OUTPUT);

  // Line sensors
  pinMode(LINE_SENSOR_FL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_FR, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BR, INPUT_PULLUP);

  // IR sensors
  pinMode(IR_REFLECT_LEFT, INPUT);
  pinMode(IR_REFLECT_CENTER, INPUT);
  pinMode(IR_REFLECT_RIGHT, INPUT);

  // Ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Bump sensors
  pinMode(BUMP_LEFT, INPUT);
  pinMode(BUMP_RIGHT, INPUT);

  // Starter switch
  pinMode(JSUMO_SWITCH, INPUT);
  updateSensorStates();
}

// Main loop
void loop() {
  int JSUMOdata = digitalRead(JSUMO_SWITCH);
  updateSensorStates();

  if (JSUMOdata) {
    if (!waitOnce) {
      waitStartTime = millis();
      waitOnce = true;
    }

    if (millis() - waitStartTime >= 3000) {
      switch (sensorStates) {
        case 0:
          snakeMovement();
          break;

        case 1:  // Front obstacle
          moveForward(255, 255);
          break;

        case 2:  // Left obstacle
          turnLeft(255, 255);
          delay(1050);
          break;

        case 3:  // Front + Left
          moveForward(155, 255);
          break;

        case 4:  // Right obstacle
          turnRight(255, 255);
          delay(1050);
          break;

        case 5:  // Front + Right
          moveForward(255, 155);
          break;

        case 8:  // Front-left line sensor
        case 16: // Front-right line sensor
        case 32: // Back-left line sensor
        case 64: // Back-right line sensor
        case 24: // Both front line sensors
        case 48: // Both back line sensors
        case 80: // Front-left + Back-left
        case 96: // Front-right + Back-right
          moveBackward(255, 255);
          delay(500);
          turnRight(255, 255);
          delay(390);
          break;

        default:
          stopMovement();
          break;
      }
    }
  }
}
