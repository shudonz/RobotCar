#include <Arduino.h>
#include <IRremote.h>
#include "SR04.h"
#include <Servo.h >

/* ---------------- IR ---------------- */
#define IR_RECEIVE_PIN 3
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

/* ---------------- ULTRASONIC ---------------- */
#define TRIG_PIN 12
#define ECHO_PIN 13
SR04 sr04(ECHO_PIN, TRIG_PIN);

/* ---------------- MOTORS ---------------- */
const int left_ctrl = 4;
const int left_pwm = 5;
const int right_ctrl = 2;
const int right_pwm = 6;

/* ---------------- SERVO ---------------- */
const int servoPin = 9;
Servo myServo;

/* ---------------- LED MATRIX ---------------- */
#define SCL_Pin A5
#define SDA_Pin A4
unsigned char C[8] = {0x3C, 0x42, 0x80, 0x80, 0x80, 0x80, 0x42, 0x3C};
unsigned char L[8] = {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFE};
unsigned char A[8] = {0x10, 0x28, 0x44, 0x82, 0x82, 0xFE, 0x82, 0x82};
unsigned char N[8] = {0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x82};
unsigned char K[8] = {0x82, 0x84, 0x88, 0xB0, 0xC0, 0xB0, 0x88, 0x84}; // Fixed 'K'
unsigned char E[8] = {0xFE, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0xFE}; // Fixed 'E'
unsigned char R[8] = {0xFE, 0x82, 0x82, 0xFE, 0x90, 0x88, 0x84, 0x82};
#define MAX_SCROLL 100
unsigned char scrollBuffer[8][MAX_SCROLL];
int scrollWidth = 0;
int scrollPos = 0;
unsigned long lastScroll = 0;

void IIC_start() {
  digitalWrite(SCL_Pin, HIGH);
  digitalWrite(SDA_Pin, HIGH);
  digitalWrite(SDA_Pin, LOW);
}
void IIC_send(unsigned char d) {
  for (char i = 0; i < 8; i++) {
    digitalWrite(SCL_Pin, LOW);
    digitalWrite(SDA_Pin, d & 1);
    digitalWrite(SCL_Pin, HIGH);
    d >>= 1;
  }
}
void IIC_end() {
  digitalWrite(SCL_Pin, LOW);
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, HIGH);
  digitalWrite(SDA_Pin, HIGH);
}
void matrix_init() {
  IIC_start();
  IIC_send(0x21);
  IIC_end();
  IIC_start();
  IIC_send(0x81);
  IIC_end();
  IIC_start();
  IIC_send(0xE7);
  IIC_end();
}
void matrix_display(unsigned char *v) {
  IIC_start();
  IIC_send(0xC0);
  for (int i = 0; i < 16; i++) IIC_send(v[i]);
  IIC_end();
  IIC_start();
  IIC_send(0x8A);
  IIC_end();
}
void setLetter(unsigned char *l, int o) {
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      scrollBuffer[r][o + c] = (l[r] & (1 << (7 - c))) ? 1 : 0;
    }
  }
}
void prepareScroll() {
  memset(scrollBuffer, 0, sizeof(scrollBuffer));
  unsigned char *letters[7] = {C, L, A, N, K, E, R};
  scrollWidth = 7 * 8 + 16;
  for (int i = 0; i < 7; i++) {
    setLetter(letters[i], i * 8);
  }
  scrollPos = 0;
}
void updateScroll() {
  unsigned char display[16] = {0};
  for (int c = 0; c < 16; c++) {
    for (int r = 0; r < 8; r++) {
      if (scrollPos + c < scrollWidth && scrollBuffer[r][scrollPos + c]) {
        display[c] |= (1 << r);
      }
    }
  }
  matrix_display(display);
  if (++scrollPos >= scrollWidth - 16) {
    scrollPos = 0;
  }
}

/* ---------------- MOTOR HELPERS ---------------- */
#define LEFT_SPEED 100
#define RIGHT_SPEED 100
#define CLEAR_THRESHOLD 30
#define MIN_CLEAR_DISTANCE 40   // Minimum acceptable distance for path selection
#define BACKUP_TIME 1800        // Time to back up after hitting obstacle (ms) - INCREASED for more separation
#define SHORT_BACKUP_RATIO 2    // Divide BACKUP_TIME by this for short backup
#define TURN_TIME_PER_45_DEG 800 // Estimated time to turn 45 degrees (ms) - SIGNIFICANTLY increased for complete rotation
#define EXTREME_ANGLE_EXTRA_TURN 600 // Extra turn time (ms) for extreme angles - DOUBLED for MAXIMUM drastic change
#define MAX_TURN_TIME 4000      // Increased timeout for longer turns
#define SCAN_CENTER_PENALTY 5   // MAXIMUM penalty - center angle almost impossible to select
#define SCAN_SIDE_BONUS 250     // MAXIMUM bonus for side angles - 150% bonus!
                                // Note: Max calculation 400cm * 250% = 1000, safe for long type
#define SCAN_INTERMEDIATE_PENALTY 30  // Even heavier penalty on intermediate angles
#define STUCK_THRESHOLD_FOR_EXTREME_ANGLES 0  // Enforce extreme angles from FIRST stuck cycle
#define PROGRESS_RESET_THRESHOLD 15  // Require MORE progress to reset stuck counter
#define STALL_COUNTER_INCREMENT 3   // Triple increment for stalls - VERY aggressive
#define EXTREME_ANGLE_LEFT 0    // Left extreme angle for drastic turns
#define EXTREME_ANGLE_RIGHT 180 // Right extreme angle for drastic turns
void front() {
  myServo.write(90);
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, LEFT_SPEED);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, RIGHT_SPEED);
}
void back() {
  myServo.write(90);
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, 180);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, 160);
}
void left() {
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, 200);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, 180);
}
void right() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, 200);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, 180);
}
void Stop() {
  analogWrite(left_pwm, 0);
  digitalWrite(left_ctrl, LOW);
  analogWrite(right_pwm, 0);
  digitalWrite(right_ctrl, LOW);
}

/* ---------------- CONTROL STATE ---------------- */
bool autoMode = false;
char driveCmd = 'S';

/* ---------------- MOVE FORWARD STALL DETECTION ---------------- */
#define INVALID_DISTANCE 999
#define MOVE_CHECK_INTERVAL 200
#define STALL_TIMEOUT 800
unsigned long lastMoveCheck = 0;
unsigned long stallStart = 0;
bool isStalled = false;
long previousDistance = INVALID_DISTANCE;
bool useShortBackup = false;  // Flag for shorter backup after failed turn
int stuckCounter = 0;  // Track how many times we've been stuck
bool wasStalled = false;  // Physical stall detection (robot stuck despite sensor showing clear) vs frontal obstacle

/* ---------------- SORT FUNCTION ---------------- */
void bubbleSort(long arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        long temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

/* ---------------- AUTO STATE MACHINE ---------------- */
enum AutoState {IDLE, MOVE_FORWARD, MOVE_BACK, SCAN, TURN_TO_CLEAR, TURN_VERIFY};
AutoState autoState = IDLE;
unsigned long stateStart = 0;
int targetAngle = 90; // 0=left, 90=center, 180=right

/* ---------------- HELPERS ---------------- */
long frontDistance() {
  long distances[5];
  for (int i = 0; i < 5; i++) {
    distances[i] = sr04.Distance();
    delay(5);
  }
  bubbleSort(distances, 5); // Sort distances array
  return distances[2];     // Return the median value
}

/* ---------------- SETUP ---------------- */
void setup() {
  Serial.begin(9600);
  pinMode(left_ctrl, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_ctrl, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  pinMode(SCL_Pin, OUTPUT);
  pinMode(SDA_Pin, OUTPUT);
  irrecv.enableIRIn();
  myServo.attach(servoPin);
  myServo.write(90); // start centered
  matrix_init();
  prepareScroll();
  Stop();
}

/* ---------------- LOOP ---------------- */
void loop() {
  unsigned long now = millis();

  // SCROLL TEXT
  if (now - lastScroll > 100) {
    updateScroll();
    lastScroll = now;
  }

  // IR INPUT
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF629D:
        driveCmd = 'F';
        autoMode = false;
        break;
      case 0xFFA857:
        driveCmd = 'B';
        autoMode = false;
        break;
      case 0xFF22DD:
        driveCmd = 'L';
        autoMode = false;
        break;
      case 0xFFC23D:
        driveCmd = 'R';
        autoMode = false;
        break;
      case 0xFF02FD:
        driveCmd = 'S';
        Stop();
        autoMode = false;
        break;
      case 0xFF6897: // * button
        autoMode = !autoMode;
        driveCmd = 'S';
        autoState = IDLE;
        Stop();
        myServo.write(90);
        stateStart = now;
        break;
    }
    irrecv.resume();
  }

  // MANUAL MODE
  if (!autoMode) {
    long fDist = frontDistance();
    bool frontBlock = (fDist < 30);
    if (driveCmd == 'F') {
      if (!frontBlock)
        front();
      else
        Stop();
    } else if (driveCmd == 'B')
      back();
    else if (driveCmd == 'L')
      left();
    else if (driveCmd == 'R')
      right();
    else
      Stop();
    return;
  }

  // AUTO MODE
  long fDist = frontDistance();
  switch (autoState) {
    case IDLE:
      if (fDist > 30) {
        autoState = MOVE_FORWARD;
        front();
      } else {
        autoState = MOVE_BACK;
        stateStart = now;
        Stop();
      }
      break;

    case MOVE_FORWARD: {
      if (fDist > 30) {
        front();

        if (now - lastMoveCheck > MOVE_CHECK_INTERVAL) {
          long currentDistance = frontDistance();
          if (abs(previousDistance - currentDistance) < 4) {
            if (!isStalled) {
              isStalled = true;
              stallStart = now;
            } else if (now - stallStart > STALL_TIMEOUT) {
              Stop();
              autoState = MOVE_BACK;
              stateStart = now;
              wasStalled = true;  // Mark that we're stuck due to stall (side wall likely)
            }
          } else {
            isStalled = false;
            // Reset stuck counter if robot is making progress
            if (stuckCounter > 0 && currentDistance > previousDistance + PROGRESS_RESET_THRESHOLD) {
              stuckCounter = 0;
            }
          }
          previousDistance = currentDistance;
          lastMoveCheck = now;
        }
      } else {
        Stop();
        autoState = MOVE_BACK;
        stateStart = now;
        wasStalled = false;  // Not a stall, we saw a wall ahead
      }
    } break;

    case MOVE_BACK: {
      back();
      unsigned long backupDuration = useShortBackup ? (BACKUP_TIME / SHORT_BACKUP_RATIO) : BACKUP_TIME;
      if (now - stateStart > backupDuration) {
        autoState = SCAN;
        stateStart = now;
        useShortBackup = false;  // Reset flag
        Stop();
      }
    } break;

    case SCAN: {
      Stop();
      delay(200);
      
      // Increment stuck counter at start of scan
      // With threshold=0: extreme angles enforced immediately from 1st stuck cycle
      // If stalled (physical obstruction), use aggressive increment to force drastic turns
      if (wasStalled) {
        stuckCounter += STALL_COUNTER_INCREMENT;  // Aggressive increment for stalls
        wasStalled = false;  // Reset flag
      } else {
        stuckCounter++;  // Normal increment for frontal obstacles
      }
      
      long distances[5];
      int angles[5] = {0, 45, 90, 135, 180};
      for (int i = 0; i < 5; i++) {
        myServo.write(angles[i]);
        delay(150);
        distances[i] = frontDistance();
      }
      
      // Apply VERY aggressive weighted scoring to force extreme turns
      long scores[5];
      int maxIndex = -1;
      long maxScore = -1;
      
      for (int i = 0; i < 5; i++) {
        scores[i] = distances[i];
        
        // VERY heavily penalize center angle (90°) - never go back to wall
        if (i == 2) {
          scores[i] = scores[i] * SCAN_CENTER_PENALTY / 100;
        }
        // When stuck multiple times, also penalize intermediate angles heavily
        // Force robot to pick extreme angles (0° or 180°) for drastic direction change
        else if ((i == 1 || i == 3) && stuckCounter > STUCK_THRESHOLD_FOR_EXTREME_ANGLES) {
          scores[i] = scores[i] * SCAN_INTERMEDIATE_PENALTY / 100;
        }
        // VERY strong bonus to extreme angles for drastic turns
        else if (i == 0 || i == 4) {
          scores[i] = scores[i] * SCAN_SIDE_BONUS / 100;
        }
        
        // Only consider directions with reasonable clearance
        if (distances[i] >= MIN_CLEAR_DISTANCE && scores[i] > maxScore) {
          maxScore = scores[i];
          maxIndex = i;
        }
      }
      
      // If no direction has sufficient clearance, pick extreme angle with most space
      // Completely exclude center and intermediate angles when stuck
      if (maxIndex == -1) {
        // When stuck, ONLY consider extreme angles (0° or 180°)
        if (stuckCounter > STUCK_THRESHOLD_FOR_EXTREME_ANGLES) {
          if (scores[0] > scores[4]) {
            maxIndex = 0;  // Hard left
          } else {
            maxIndex = 4;  // Hard right
          }
        } else {
          // First time stuck, allow non-center angles
          for (int i = 0; i < 5; i++) {
            if (i != 2 && scores[i] > maxScore) {
              maxScore = scores[i];
              maxIndex = i;
            }
          }
        }
      }
      
      // Failsafe: if still no valid direction, alternate between hard left and hard right
      if (maxIndex == -1) {
        maxIndex = (stuckCounter % 2 == 0) ? 0 : 4;
      }
      
      targetAngle = angles[maxIndex];
      autoState = TURN_TO_CLEAR;
      stateStart = now;
    } break;

    case TURN_TO_CLEAR: {
      // Calculate required turn time based on angle difference from center
      int angleFromCenter = abs(targetAngle - 90);
      // Note: Integer division is exact since targetAngle is always a multiple of 45
      // (scan angles are 0, 45, 90, 135, 180)
      unsigned long requiredTurnTime = ((unsigned long)angleFromCenter * TURN_TIME_PER_45_DEG) / 45;
      
      // Add EXTRA turn time for extreme angles (0° or 180°) to ensure VERY drastic direction change
      if (targetAngle == EXTREME_ANGLE_LEFT || targetAngle == EXTREME_ANGLE_RIGHT) {
        requiredTurnTime += EXTREME_ANGLE_EXTRA_TURN;
      }
      
      // Turn in the appropriate direction for the calculated time
      if (now - stateStart < requiredTurnTime) {
        if (targetAngle < 90) {
          left();  // Turn left for angles 0-89
        } else if (targetAngle > 90) {
          right(); // Turn right for angles 91-180
        } else {
          Stop(); // Should not happen as center is heavily penalized
        }
      } else {
        // Done turning, move to verification state
        Stop();
        myServo.write(90); // Reset servo to center
        autoState = TURN_VERIFY;
        stateStart = now;
      }
      
      // Timeout safety - if taking too long, reset
      if (now - stateStart > MAX_TURN_TIME) {
        Stop();
        myServo.write(90);
        autoState = IDLE;
      }
    } break;
    
    case TURN_VERIFY: {
      // Wait briefly for servo to settle and distance reading to stabilize
      if (now - stateStart > 300) {
        long clearDistance = frontDistance();
        if (clearDistance > CLEAR_THRESHOLD) {
          // Path is clear, proceed forward
          autoState = MOVE_FORWARD;
          isStalled = false;  // Reset stall detection
          previousDistance = INVALID_DISTANCE;
        } else {
          // Path still blocked after turning, use shorter backup and rescan
          useShortBackup = true;
          autoState = MOVE_BACK;
          stateStart = now;
        }
      }
    } break;
  }
}