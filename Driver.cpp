#include <Arduino.h>

// Pin definitions
//Motors: Digital output
#define M1L 11
#define M1R 10
#define M2L 5
#define M2R 6

//Starter switch: Digital input
#define JSUMO_SWITCH 4

// Line sensors: Digital input
#define LINE_SENSOR_FL 2  // front-left
#define LINE_SENSOR_FR 3  // front-right
#define LINE_SENSOR_BL 12 // back-left
#define LINE_SENSOR_BR 13 // back-right

//Bump sensors
#define BUMP_LEFT 7
#define BUMP_RIGHT 8

//IR sensors: Use	Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Use Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int ONstate = 1;
const int FRONT_BIT   = 0;
const int LEFT_BIT    = 1;
const int RIGHT_BIT   = 2;
const int LINE_FR_BIT = 3;
const int LINE_FL_BIT = 4;
const int LINE_BR_BIT = 5;
const int LINE_BL_BIT = 6;


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

void setup() {
  Serial.begin(9600);
  //Motor pins
  pinMode(M2R, OUTPUT);
  pinMode(M2L, OUTPUT);
  pinMode(M1L, OUTPUT);
  pinMode(M1R, OUTPUT);

  //Line sensors pins
  pinMode(LINE_SENSOR_FL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_FR, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BR, INPUT_PULLUP);
  
  //IR sensors pins
  pinMode(IR_REFLECT_LEFT, INPUT);
  pinMode(IR_REFLECT_CENTER, INPUT);
  pinMode(IR_REFLECT_RIGHT, INPUT);

  //Ultrasonic sensor pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  //Bump sensors pins
  pinMode(BUMP_LEFT, INPUT);
  pinMode(BUMP_RIGHT, INPUT);

  //Starter switch pin
  pinMode(JSUMO_SWITCH, INPUT);
  updateSensorStates();
}

void moveForward(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed); // left forward
  analogWrite(M2R, rightSpeed); // right forward
  analogWrite(M1R, 0); // left backward off
  analogWrite(M2L, 0); // right backward off
}

void moveBackward(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed); // left backward
  analogWrite(M2L, rightSpeed); // right backward
  analogWrite(M1L, 0); // left forward off
  analogWrite(M2R, 0); // right forward off
}

void turnLeft(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);  // left backward
  analogWrite(M2R, rightSpeed); // right forward
  analogWrite(M1L, 0); 
  analogWrite(M2L, 0);
}

void turnRight(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);  // left forward
  analogWrite(M2L, rightSpeed); // right backward
  analogWrite(M1R, 0);
  analogWrite(M2R, 0);
}

void stopMovement() {
  analogWrite(M1L, 0);
  analogWrite(M1R, 0);
  analogWrite(M2L, 0);
  analogWrite(M2R, 0);
}

void snakeMovement() {
  static bool direction = true;
  static unsigned long lastChangeTime = 0;
  static unsigned long actionStartTime = 0;

  unsigned long currentTime = millis();

  // Switch direction every 500ms
  if (currentTime - lastChangeTime >= 500) {
    direction = !direction;
    lastChangeTime = currentTime;
    actionStartTime = currentTime; // Reset movement timer
  }

  // Move forward in snaking pattern
  if (direction) {
    moveForward(255, 150);
  } else {
    moveForward(150, 255);
  }

  // Sensor break condition
  if (!digitalRead(LINE_SENSOR_FR) || !digitalRead(LINE_SENSOR_FL) || 
      digitalRead(BUMP_LEFT) || digitalRead(BUMP_RIGHT)) {
    stopMovement();
    return;
  }

  // Stop after 950ms
  if (currentTime - actionStartTime > 950) {
    stopMovement();
  }
}



void updateSensorStates() {
  sensorStates = (analogRead(IR_REFLECT_CENTER) > 500 << FRONT_BIT) |
                 (analogRead(IR_REFLECT_LEFT) > 500 << LEFT_BIT) |
                 (analogRead(IR_REFLECT_RIGHT) > 500 << RIGHT_BIT) |
                 (!digitalRead(LINE_SENSOR_FR) << LINE_FR_BIT) |
                 (!digitalRead(LINE_SENSOR_FL) << LINE_FL_BIT) |
                 (!digitalRead(LINE_SENSOR_BR) << LINE_BR_BIT) |
                 (!digitalRead(LINE_SENSOR_BL) << LINE_BL_BIT);
}


unsigned long newc = 0;

void loop() {
  int JSUMOdata = digitalRead(JSUMO_SWITCH);
  updateSensorStates();
  newc = millis();

  if (JSUMOdata) {
    if (!waitOnce) {
      waitStartTime = millis();
      waitOnce = true;
    }

    if (millis() - waitStartTime >= 3000) {
      switch (sensorStates) {
        case 0:  // No obstacle
          snakeMovement();
          break;

        case 1:  // Obstacle in front (IR center)
          moveForward(255, 255);
          Serial.println("forward");
          break;

        case 2:  // Obstacle on left (IR left)
          turnLeft(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 1050) {
            if (!digitalRead(LINE_SENSOR_FL) || !digitalRead(LINE_SENSOR_FR) ||
                analogRead(IR_REFLECT_CENTER) > 500) break;
          }
          break;

        case 3:  // Obstacle front + left
          moveForward(155, 255);
          break;

        case 4:  // Obstacle on right (IR right)
          turnRight(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 1050) {
            if (!digitalRead(LINE_SENSOR_FL) || !digitalRead(LINE_SENSOR_FR) ||
                analogRead(IR_REFLECT_CENTER) > 500) break;
          }
          break;

        case 5:  // Obstacle front + right
          moveForward(255, 155);
          break;

        case 24:  // Both line sensors (bit 3 and 4)
          moveBackward(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 500) {
            if (analogRead(IR_REFLECT_CENTER) > 500 ||
                analogRead(IR_REFLECT_LEFT) > 500 ||
                analogRead(IR_REFLECT_RIGHT) > 500) break;
          }

          turnRight(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 390) {
            if (analogRead(IR_REFLECT_CENTER) > 500 ||
                analogRead(IR_REFLECT_LEFT) > 500 ||
                analogRead(IR_REFLECT_RIGHT) > 500) break;
          }
          break;

        case 8:  // Right line sensor (LINE_SENSOR_FR)
          moveBackward(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 500) {
            if (analogRead(IR_REFLECT_CENTER) > 500 ||
                analogRead(IR_REFLECT_LEFT) > 500 ||
                analogRead(IR_REFLECT_RIGHT) > 500) break;
          }

          turnRight(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 390) {
            if (analogRead(IR_REFLECT_CENTER) > 500 ||
                analogRead(IR_REFLECT_LEFT) > 500 ||
                analogRead(IR_REFLECT_RIGHT) > 500) break;
          }
          break;

        case 16:  // Left line sensor (LINE_SENSOR_FL)
          moveBackward(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 500) {
            if (analogRead(IR_REFLECT_CENTER) > 500 ||
                analogRead(IR_REFLECT_LEFT) > 500 ||
                analogRead(IR_REFLECT_RIGHT) > 500) break;
          }

          turnLeft(255, 255);
          actionStartTime = millis();
          while (millis() - actionStartTime < 390) {
            if (analogRead(IR_REFLECT_CENTER) > 500 ||
                analogRead(IR_REFLECT_LEFT) > 500 ||
                analogRead(IR_REFLECT_RIGHT) > 500) break;
          }
          break;

        case 25:
        case 17:
        case 9:
        case 26:
        case 27:
        case 28:
        case 29:
          stopMovement();
          break;
      }
    }
  }
}
