#include <BluetoothSerial.h>
#include <ESP32Servo.h>  // Include ESP32 servo library

// Motor_driver_1 - Right side motors
const int top_right_Motor_input_1 = 33;
const int top_right_Motor_input_2 = 31;
const int low_right_Motor_input_3 = 30;
const int low_right_Motor_input_4 = 29;

// Motor_driver_2 - Left side motors
const int top_left_Motor_input_1 = 9;
const int top_left_Motor_input_2 = 10;
const int low_left_Motor_input_3 = 11;
const int low_left_Motor_input_4 = 12;

// ===== IR_ARRAY_sensor_pins =====
const int ir_1 = 4;
const int ir_2 = 5;
const int ir_3 = 6;
const int ir_4 = 7;
const int ir_5 = 34;

// ===== LDR Sensor =====
const int ldrPin = 37;  // LDR on pin 37 (moved from servo)
const int LDR_THRESHOLD = 500;  // Adjust this value based on testing

// ===== Servo Motor =====
const int servoPin = 3;  // Servo moved to pin 3
Servo myServo;  // Create servo object
int servoAngle = 90;  // Default center position

// ===== Motor speed (PWM 0-1023)
const int motorSpeed = 500;

// === PWM channels =====
const int EN_top_Left_motor = 8;
const int EN_low_Left_motor = 13;
const int EN_top_right_motor = 27;
const int EN_low_right_motor = 28;

// ===== PWM channels for enable pins =====
const int channelTopLeft = 0;
const int channelLowLeft = 1;
const int channelTopRight = 2;
const int channelLowRight = 3;

// ===== Bluetooth =====
BluetoothSerial SerialBT;
char command;

// Control mode
bool autoMode = true;
bool ldrTriggered = false;

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Car");
  
  // Setup motor control pins as outputs
  pinMode(top_right_Motor_input_1, OUTPUT);
  pinMode(top_right_Motor_input_2, OUTPUT);
  pinMode(low_right_Motor_input_3, OUTPUT);
  pinMode(low_right_Motor_input_4, OUTPUT);
  pinMode(top_left_Motor_input_1, OUTPUT);
  pinMode(top_left_Motor_input_2, OUTPUT);
  pinMode(low_left_Motor_input_3, OUTPUT);
  pinMode(low_left_Motor_input_4, OUTPUT);
  
  // Setup PWM channels for enable pins
  ledcSetup(channelTopLeft, 5000, 10);
  ledcSetup(channelLowLeft, 5000, 10);
  ledcSetup(channelTopRight, 5000, 10);
  ledcSetup(channelLowRight, 5000, 10);
  
  // Attach enable pins to PWM channels
  ledcAttachPin(EN_top_Left_motor, channelTopLeft);
  ledcAttachPin(EN_low_Left_motor, channelLowLeft);
  ledcAttachPin(EN_top_right_motor, channelTopRight);
  ledcAttachPin(EN_low_right_motor, channelLowRight);
  
  // Set IR sensor pins as inputs
  pinMode(ir_1, INPUT);
  pinMode(ir_2, INPUT);
  pinMode(ir_3, INPUT);
  pinMode(ir_4, INPUT);
  pinMode(ir_5, INPUT);
  
  // Setup LDR pin
  pinMode(ldrPin, INPUT);
  
  // Setup Servo Motor
  myServo.attach(servoPin);  // Attach servo to pin 3
  myServo.write(servoAngle); // Center the servo
  
  stopMotors();
  
  Serial.println("Line Following Robot Started");
  Serial.println("Mode: AUTO LINE FOLLOWING");
  Serial.println("Commands: F=Forward, B=Back, L=Left, R=Right, S=Stop, X=Restart Auto");
  Serial.println("Servo Commands (Manual only): 1=Left, 2=Center, 3=Right");
  Serial.println("LDR Safety: Stops car when light detected");
}

// ===== Motor Functions =====
void forward() {
  if (ldrTriggered) return;  // Skip if LDR triggered
  digitalWrite(top_right_Motor_input_1, HIGH);
  digitalWrite(top_right_Motor_input_2, LOW);
  digitalWrite(low_right_Motor_input_3, HIGH);
  digitalWrite(low_right_Motor_input_4, LOW);
  digitalWrite(top_left_Motor_input_1, HIGH);
  digitalWrite(top_left_Motor_input_2, LOW);
  digitalWrite(low_left_Motor_input_3, HIGH);
  digitalWrite(low_left_Motor_input_4, LOW);
  ledcWrite(channelTopLeft, motorSpeed);
  ledcWrite(channelLowLeft, motorSpeed);
  ledcWrite(channelTopRight, motorSpeed);
  ledcWrite(channelLowRight, motorSpeed);
}

void backward() {
  if (ldrTriggered) return;  // Skip if LDR triggered
  digitalWrite(top_right_Motor_input_1, LOW);
  digitalWrite(top_right_Motor_input_2, HIGH);
  digitalWrite(low_right_Motor_input_3, LOW);
  digitalWrite(low_right_Motor_input_4, HIGH);
  digitalWrite(top_left_Motor_input_1, LOW);
  digitalWrite(top_left_Motor_input_2, HIGH);
  digitalWrite(low_left_Motor_input_3, LOW);
  digitalWrite(low_left_Motor_input_4, HIGH);
  ledcWrite(channelTopLeft, motorSpeed);
  ledcWrite(channelLowLeft, motorSpeed);
  ledcWrite(channelTopRight, motorSpeed);
  ledcWrite(channelLowRight, motorSpeed);
}

void turnLeft() {
  if (ldrTriggered) return;  // Skip if LDR triggered
  digitalWrite(top_right_Motor_input_1, HIGH);
  digitalWrite(top_right_Motor_input_2, LOW);
  digitalWrite(low_right_Motor_input_3, HIGH);
  digitalWrite(low_right_Motor_input_4, LOW);
  digitalWrite(top_left_Motor_input_1, LOW);
  digitalWrite(top_left_Motor_input_2, HIGH);
  digitalWrite(low_left_Motor_input_3, LOW);
  digitalWrite(low_left_Motor_input_4, HIGH);
  ledcWrite(channelTopLeft, motorSpeed);
  ledcWrite(channelLowLeft, motorSpeed);
  ledcWrite(channelTopRight, motorSpeed);
  ledcWrite(channelLowRight, motorSpeed);
}

void turnRight() {
  if (ldrTriggered) return;  // Skip if LDR triggered
  digitalWrite(top_right_Motor_input_1, LOW);
  digitalWrite(top_right_Motor_input_2, HIGH);
  digitalWrite(low_right_Motor_input_3, LOW);
  digitalWrite(low_right_Motor_input_4, HIGH);
  digitalWrite(top_left_Motor_input_1, HIGH);
  digitalWrite(top_left_Motor_input_2, LOW);
  digitalWrite(low_left_Motor_input_3, HIGH);
  digitalWrite(low_left_Motor_input_4, LOW);
  ledcWrite(channelTopLeft, motorSpeed);
  ledcWrite(channelLowLeft, motorSpeed);
  ledcWrite(channelTopRight, motorSpeed);
  ledcWrite(channelLowRight, motorSpeed);
}

void stopMotors() {
  digitalWrite(top_right_Motor_input_1, LOW);
  digitalWrite(top_right_Motor_input_2, LOW);
  digitalWrite(low_right_Motor_input_3, LOW);
  digitalWrite(low_right_Motor_input_4, LOW);
  digitalWrite(top_left_Motor_input_1, LOW);
  digitalWrite(top_left_Motor_input_2, LOW);
  digitalWrite(low_left_Motor_input_3, LOW);
  digitalWrite(low_left_Motor_input_4, LOW);
  ledcWrite(channelTopLeft, 0);
  ledcWrite(channelLowLeft, 0);
  ledcWrite(channelTopRight, 0);
  ledcWrite(channelLowRight, 0);
}

// Check LDR sensor
bool checkLDR() {
  int ldrValue = analogRead(ldrPin);
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  
  if (ldrValue > LDR_THRESHOLD) {
    if (!ldrTriggered) {
      Serial.println("LDR TRIGGERED! Light detected - EMERGENCY STOP");
      Serial.println("All manual commands disabled until light is gone");
      stopMotors();
    }
    return true;
  } else {
    if (ldrTriggered) {
      Serial.println("LDR CLEARED: Light gone - Manual control restored");
    }
    return false;
  }
}

// Servo control function (manual mode only)
void controlServo(char servoCommand) {
  if (ldrTriggered) {
    Serial.println("LDR triggered - Servo control disabled");
    return;
  }
  
  switch(servoCommand) {
    case '1': // Left
      servoAngle = 0;
      myServo.write(servoAngle);
      Serial.println("Servo: LEFT (0°)");
      break;
    case '2': // Center
      servoAngle = 90;
      myServo.write(servoAngle);
      Serial.println("Servo: CENTER (90°)");
      break;
    case '3': // Right
      servoAngle = 180;
      myServo.write(servoAngle);
      Serial.println("Servo: RIGHT (180°)");
      break;
  }
}

// Check if any sensor detects a line
bool isLineDetected() {
  return (digitalRead(ir_1) == 1 || digitalRead(ir_2) == 1 || 
          digitalRead(ir_3) == 1 || digitalRead(ir_4) == 1 || 
          digitalRead(ir_5) == 1);
}

void lineFollowingLogic() {
  int val1 = digitalRead(ir_1);
  int val2 = digitalRead(ir_2);
  int val3 = digitalRead(ir_3);
  int val4 = digitalRead(ir_4);
  int val5 = digitalRead(ir_5);
  
  if (val3 == 1) {
    forward();
  } else if (val1 == 1 || val2 == 1) {
    turnLeft();
  } else if (val4 == 1 || val5 == 1) {
    turnRight();
  } else {
    stopMotors();
  }
}

// Switch to manual mode
void switchToManualMode() {
  autoMode = false;
  stopMotors();
  Serial.println("SWITCHED TO MANUAL MODE");
  Serial.println("Use Bluetooth commands: F, B, L, R, S, X=Restart Auto");
  Serial.println("Servo: 1=Left, 2=Center, 3=Right");
}

// Restart auto mode
void restartAutoMode() {
  autoMode = true;
  ldrTriggered = false;  // Reset LDR trigger when restarting auto
  stopMotors();
  myServo.write(90);  // Center servo when returning to auto mode
  Serial.println("RESTARTING AUTO MODE");
  Serial.println("Searching for line...");
}

// ===== Main Loop =====
void loop() {
  // Check LDR sensor (works in both modes)
  ldrTriggered = checkLDR();
  
  if (!autoMode) {
    // Manual Bluetooth control
    if (SerialBT.available()) {
      command = SerialBT.read();
      
      // Check if LDR is triggered - block all movement commands
      if (ldrTriggered && (command == 'F' || command == 'B' || command == 'L' || command == 'R')) {
        Serial.println("Command blocked: LDR triggered - emergency stop active");
        stopMotors();
        return;
      }
      
      switch(command) {
        case 'F': forward(); break;
        case 'B': backward(); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        case 'S': stopMotors(); break;
        case 'X': restartAutoMode(); break;
        case '1': controlServo('1'); break;  // Servo left
        case '2': controlServo('2'); break;  // Servo center
        case '3': controlServo('3'); break;  // Servo right
      }
    }
    delay(5);  // Faster for manual control
  } else {
    // Auto Line Following - Servo not used in auto mode
    // LDR doesn't affect auto mode, only manual
    if (isLineDetected()) {
      lineFollowingLogic();
    } else {
      // Immediately switch to manual mode when line is lost
      switchToManualMode();
    }
    delay(50);  // Slower for stable line following
  }
}