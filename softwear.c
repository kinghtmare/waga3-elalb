omar mohamed :
/*
 * Line Following Robot with 5 IR Sensors
 * Uses 5 IR sensors to follow a line and move in different directions
 *
 * IR Sensors (5 sensors in front of car):
 * - IR1: GPIO32 (Far Right)
 * - IR2: GPIO33 (Right)
 * - IR3: GPIO25 (Center)
 * - IR4: GPIO__ (Left) - PIN NUMBER TO BE FILLED
 * - IR5: GPIO__ (Far Left) - PIN NUMBER TO BE FILLED
 *
 * Motor Pins:
 * - Motor1: GPIO3,23,15,2 (EN:21,22)
 * - Motor2: GPIO17,5,18,19 (EN:4,16)
 *
 * Bluetooth Commands:
 * - F = Forward
 * - B = Backward
 * - L = Left turn
 * - R = Right turn
 * - S = Stop
 * - A = Auto line following
 * - M = Manual mode
 * - H = Help
 */

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

    // ===== IR SENSOR PINS =====
    const int IR1 = 32; // GPIO32 (Far Right)
const int IR2 = 33;     // GPIO33 (Right)
const int IR3 = 25;     // GPIO25 (Center)
const int IR4 = 0;      // GPIO__ (Left) - FILL IN PIN NUMBER
const int IR5 = 0;      // GPIO__ (Far Left) - FILL IN PIN NUMBER

// ===== MOTOR PINS =====
// Motor 1 (Left side)
const int MOTOR1_IN1 = 3;
const int MOTOR1_IN2 = 23;
const int MOTOR1_IN3 = 15;
const int MOTOR1_IN4 = 2;
const int MOTOR1_EN1 = 21;
const int MOTOR1_EN2 = 22;

// Motor 2 (Right side)
const int MOTOR2_IN1 = 17;
const int MOTOR2_IN2 = 5;
const int MOTOR2_IN3 = 18;
const int MOTOR2_IN4 = 19;
const int MOTOR2_EN1 = 4;
const int MOTOR2_EN2 = 16;

// ===== SERVO CONFIG =====
const int SERVO_PIN = 26; // GPIO26 for servo control (PWM capable, not used)
Servo panServo;

// ===== CONFIGURATION =====
const int IR_THRESHOLD = 2000;      // ADC threshold (0-4095)
const int LINE_SPEED = 150;         // Speed for line following
const int TURN_SPEED = 180;         // Speed for turning (increased for better visibility)
const int MANUAL_SPEED = 180;       // Speed for manual control
const int COMMAND_TIMEOUT = 3000;   // Auto-stop timeout
const bool SENSOR_INVERTED = false; // Set to false if robot moves on everything but black

// ===== GLOBAL VARIABLES =====
BluetoothSerial SerialBT;
bool autoMode = false;
unsigned long lastCommandTime = 0;
unsigned long lastSensorRead = 0;
char lastCommand = 'S'; // Track last command type

// ===== SETUP =====
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Line Following Robot Starting...");

    // Setup IR sensor pins
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);

    // Setup motor pins
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR1_IN3, OUTPUT);
    pinMode(MOTOR1_IN4, OUTPUT);
    pinMode(MOTOR1_EN1, OUTPUT);
    pinMode(MOTOR1_EN2, OUTPUT);

    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    pinMode(MOTOR2_EN1, OUTPUT);
    pinMode(MOTOR2_EN2, OUTPUT);

    // Setup servo
    Serial.println("Setting up servo on pin " + String(SERVO_PIN));
    panServo.attach(SERVO_PIN);
    delay(1000);        // Give servo time to initialize
    panServo.write(90); // Center position
    Serial.println("Servo initialized at 90°");

    // Start Bluetooth
    if (!SerialBT.begin("LineRobot"))
    {
        Serial.println("ERROR: Bluetooth failed to start!");
    }
    else
    {
        Serial.println("Bluetooth started: LineRobot");
        SerialBT.println("=== LINE FOLLOWING ROBOT ===");
        SerialBT.println("5 IR Sensor Line Following");
        SerialBT.println("Commands: F, B, L, R, S, A, M, H");
    }

    stopAll();
    lastCommandTime = millis();

    Serial.println("Line Following Robot Ready!");
    printHelp();
}

// ===== MOTOR CONTROL =====
void setMotor1Dir(int in1, int in2, int in3, int in4)
{
    digitalWrite(MOTOR1_IN1, in1);
    digitalWrite(MOTOR1_IN2, in2);
    digitalWrite(MOTOR1_IN3, in3);
    digitalWrite(MOTOR1_IN4, in4);
}

void setMotor2Dir(int in1, int in2, int in3, int in4)
{
    digitalWrite(MOTOR2_IN1, in1);
    digitalWrite(MOTOR2_IN2, in2);
    digitalWrite(MOTOR2_IN3, in3);
    digitalWrite(MOTOR2_IN4, in4);
}

void setMotor1Speed(int speed)
{
    speed = constrain(speed, 0, 255);
    analogWrite(MOTOR1_EN1, speed);
    analogWrite(MOTOR1_EN2, speed);
}

void setMotor2Speed(int speed)
{
    speed = constrain(speed, 0, 255);
    analogWrite(MOTOR2_EN1, speed);
    analogWrite(MOTOR2_EN2, speed);
}

void moveForward()
{
    setMotor1Dir(HIGH, LOW, HIGH, LOW);
    setMotor2Dir(HIGH, LOW, HIGH, LOW);
    setMotor1Speed(MANUAL_SPEED);
    setMotor2Speed(MANUAL_SPEED);
}

void moveBackward()
{
    setMotor1Dir(LOW, HIGH, LOW, HIGH);
    setMotor2Dir(LOW, HIGH, LOW, HIGH);
    setMotor1Speed(MANUAL_SPEED);
    setMotor2Speed(MANUAL_SPEED);
}

void turnLeft()
{
    SerialBT.println("DEBUG: turnLeft() called");
    // Left turn: Left motor forward, Right motor backward (swapped)
    setMotor1Dir(HIGH, LOW, HIGH, LOW); // Left motor forward
    setMotor2Dir(LOW, HIGH, LOW, HIGH); // Right motor backward
    setMotor1Speed(TURN_SPEED);
    setMotor2Speed(TURN_SPEED);
    SerialBT.println("DEBUG: Motor1: HIGH,LOW,HIGH,LOW Speed:" + String(TURN_SPEED));
    SerialBT.println("DEBUG: Motor2: LOW,HIGH,LOW,HIGH Speed:" + String(TURN_SPEED));
}

void turnRight()
{
    SerialBT.println("DEBUG: turnRight() called");
    // Right turn: Left motor backward, Right motor forward (swapped)
    setMotor1Dir(LOW, HIGH, LOW, HIGH); // Left motor backward
    setMotor2Dir(HIGH, LOW, HIGH, LOW); // Right motor forward
    setMotor1Speed(TURN_SPEED);
    setMotor2Speed(TURN_SPEED);
    SerialBT.println("DEBUG: Motor1: LOW,HIGH,LOW,HIGH Speed:" + String(TURN_SPEED));
    SerialBT.println("DEBUG: Motor2: HIGH,LOW,HIGH,LOW Speed:" + String(TURN_SPEED));
}

void stopAll()
{
    setMotor1Dir(LOW, LOW, LOW, LOW);
    setMotor2Dir(LOW, LOW, LOW, LOW);
    setMotor1Speed(0);
    setMotor2Speed(0);
}

// ===== LINE FOLLOWING =====
bool isLineDetected()
{
    int sensor1 = analogRead(IR1);
    int sensor2 = analogRead(IR2);
    int sensor3 = analogRead(IR3);
    int sensor4 = analogRead(IR4);
    int sensor5 = analogRead(IR5);

    bool line1 = (sensor1 >= IR_THRESHOLD);
    bool line2 = (sensor2 >= IR_THRESHOLD);
    bool line3 = (sensor3 >= IR_THRESHOLD);
    bool line4 = (sensor4 >= IR_THRESHOLD);
    bool line5 = (sensor5 >= IR_THRESHOLD);

    // Invert sensor readings if LEDs go off on black line
    if (SENSOR_INVERTED)
    {
        line1 = !line1;
        line2 = !line2;
        line3 = !line3;
        line4 = !line4;
        line5 = !line5;
    }

    return (line1 && line2 && line3 && line4 && line5);
}

void followLine()
{
    int sensor1 = analogRead(IR1);
    int sensor2 = analogRead(IR2);
    int sensor3 = analogRead(IR3);
    int sensor4 = analogRead(IR4);
    int sensor5 = analogRead(IR5);

    bool line1 = (sensor1 >= IR_THRESHOLD);
    bool line2 = (sensor2 >= IR_THRESHOLD);
    bool line3 = (sensor3 >= IR_THRESHOLD);
    bool line4 = (sensor4 >= IR_THRESHOLD);
    bool line5 = (sensor5 >= IR_THRESHOLD);

    // Invert sensor readings if LEDs go off on black line
    if (SENSOR_INVERTED)
    {
        line1 = !line1;
        line2 = !line2;
        line3 = !line3;
        line4 = !line4;
        line5 = !line5;
    }

    // Center sensor on line: go straight
    if (line3 && !line1 && !line2 && !line4 && !line5)
    {
        setMotor1Dir(HIGH, LOW, HIGH, LOW);
        setMotor2Dir(HIGH, LOW, HIGH, LOW);
        setMotor1Speed(LINE_SPEED);
        setMotor2Speed(LINE_SPEED);
        SerialBT.println("Center: Going straight");
    }
    // Far right sensor detects line: sharp right turn
    else if (line1 && !line2 && !line3 && !line4 && !line5)
    {
        setMotor1Dir(HIGH, LOW, HIGH, LOW);
        setMotor2Dir(LOW, HIGH, LOW, HIGH);
        setMotor1Speed(LINE_SPEED);
        setMotor2Speed(TURN_SPEED);
        SerialBT.println("Far Right: Sharp right turn");
    }
    // Right sensor detects line: right turn
    else if (line2 && !line1 && !line3 && !line4 && !line5)
    {
        setMotor1Dir(HIGH, LOW, HIGH, LOW);
        setMotor2Dir(LOW, HIGH, LOW, HIGH);
        setMotor1Speed(LINE_SPEED);

        setMotor2Speed(TURN_SPEED);
        SerialBT.println("Right: Turning right");
    }
    // Left sensor detects line: left turn
    else if (line4 && !line1 && !line2 && !line3 && !line5)
    {
        setMotor1Dir(LOW, HIGH, LOW, HIGH);
        setMotor2Dir(HIGH, LOW, HIGH, LOW);
        setMotor1Speed(TURN_SPEED);
        setMotor2Speed(LINE_SPEED);
        SerialBT.println("Left: Turning left");
    }
    // Far left sensor detects line: sharp left turn
    else if (line5 && !line1 && !line2 && !line3 && !line4)
    {
        setMotor1Dir(LOW, HIGH, LOW, HIGH);
        setMotor2Dir(HIGH, LOW, HIGH, LOW);
        setMotor1Speed(TURN_SPEED);
        setMotor2Speed(LINE_SPEED);
        SerialBT.println("Far Left: Sharp left turn");
    }
    // Multiple sensors: go straight
    else if ((line1 && line2)(line2 && line3)(line3 && line4)(line4 && line5)(line1 && line2 && line3)(line2 && line3 && line4)(line3 && line4 && line5))
    {
        setMotor1Dir(HIGH, LOW, HIGH, LOW);
        setMotor2Dir(HIGH, LOW, HIGH, LOW);
        setMotor1Speed(LINE_SPEED);
        setMotor2Speed(LINE_SPEED);
        SerialBT.println("Multiple: Going straight");
    }
    // No line detected: search
    else
    {
        setMotor1Dir(LOW, HIGH, LOW, HIGH);
        setMotor2Dir(HIGH, LOW, HIGH, LOW);
        setMotor1Speed(TURN_SPEED);
        setMotor2Speed(TURN_SPEED);
        SerialBT.println("No line: Searching...");
    }
}

// ===== BLUETOOTH COMMANDS =====
void handleBluetoothCommand(char cmd)
{
    cmd = toupper(cmd);
    lastCommandTime = millis();
    lastCommand = cmd; // Store the command type

    switch (cmd)
    {
    case 'F':
        if (!autoMode)
        {
            moveForward();
            SerialBT.println("Moving forward");
        }
        break;

    case 'B':
        if (!autoMode)
        {
            moveBackward();
            SerialBT.println("Moving backward");
        }
        break;

    case 'L':
        if (!autoMode)
        {
            SerialBT.println("DEBUG: L command received, calling turnLeft()");
            turnLeft();
            SerialBT.println("Turning left for 5 seconds...");
            delay(5000); // Turn for 5 seconds
            stopAll();
            SerialBT.println("Turn complete");
        }
        else
        {
            SerialBT.println("DEBUG: L command ignored - in AUTO mode");
        }
        break;

    case 'R':
        if (!autoMode)
        {
            SerialBT.println("DEBUG: R command received, calling turnRight()");
            turnRight();
            SerialBT.println("Turning right for 5 seconds...");
            delay(5000); // Turn for 5 seconds
            stopAll();
            SerialBT.println("Turn complete");
        }
        else
        {
            SerialBT.println("DEBUG: R command ignored - in AUTO mode");
        }
        break;

    case 'S':
        stopAll();
        SerialBT.println("Stopped");
        break;

    case 'A':
        autoMode = true;
        SerialBT.println("Auto line following ON");
        break;

    case 'M':
        autoMode = false;
        stopAll();
        SerialBT.println("Manual mode ON");
        break;

    case 'H':
        printHelp();
        break;

    case 'U':
    case 'u':
        SerialBT.println("Moving servo to 0°...");
        panServo.write(0); // Up position
        delay(500);        // Give servo time to move
        SerialBT.println("Servo: Up (0°)");
        break;

    case 'D':
    case 'd':
        SerialBT.println("Moving servo to 180°...");
        panServo.write(180); // Down position
        delay(500);          // Give servo time to move
        SerialBT.println("Servo: Down (180°)");
        break;

    case 'C':
    case 'c':
        SerialBT.println("Moving servo to 90°...");
        panServo.write(90); // Center position
        delay(500);         // Give servo time to move
        SerialBT.println("Servo: Center (90°)");
        break;

    default:
        // Check if it's a number (0-9) for servo angle
        if (cmd >= '0' && cmd <= '9')
        {
            int angle = map(cmd - '0', 0, 9, 0, 180);

            SerialBT.println("Moving servo to " + String(angle) + "°...");
            panServo.write(angle);
            delay(500); // Give servo time to move
            SerialBT.println("Servo: " + String(angle) + "°");
        }
        else
        {
            SerialBT.println("Unknown command: " + String(cmd));
            SerialBT.println("Send 'H' for help");
        }
        break;
    }
}

// ===== HELP =====
void printHelp()
{
    SerialBT.println("=== LINE FOLLOWING ROBOT ===");
    SerialBT.println("F = Forward");
    SerialBT.println("B = Backward");
    SerialBT.println("L = Left turn");
    SerialBT.println("R = Right turn");
    SerialBT.println("S = Stop");
    SerialBT.println("A = Auto line following");
    SerialBT.println("M = Manual mode");
    SerialBT.println("U = Servo Up (0°)");
    SerialBT.println("D = Servo Down (180°)");
    SerialBT.println("C = Servo Center (90°)");
    SerialBT.println("0-9 = Servo angle (0=0°, 5=90°, 9=180°)");
    SerialBT.println("H = Help");
    SerialBT.println("Current mode: " + String(autoMode ? "AUTO" : "MANUAL"));
}

// ===== MAIN LOOP =====
void loop()
{
    // Handle Bluetooth commands
    if (SerialBT.available())
    {
        char cmd = SerialBT.read();
        handleBluetoothCommand(cmd);
    }

    // Auto line following mode
    if (autoMode)
    {
        if (isLineDetected())
        {
            followLine();
        }
        else
        {
            // No line detected - keep searching
            setMotor1Dir(LOW, HIGH, LOW, HIGH);
            setMotor2Dir(HIGH, LOW, HIGH, LOW);
            setMotor1Speed(TURN_SPEED);
            setMotor2Speed(TURN_SPEED);
        }
    }

    // Auto-stop in manual mode (only for F/B commands, not L/R)
    // Note: L/R commands don't auto-stop to allow continuous turning
    if (!autoMode && millis() - lastCommandTime > COMMAND_TIMEOUT && lastCommandTime > 0)
    {
        // Only auto-stop if it's not a turning command (L/R)
        if (lastCommand != 'L' && lastCommand != 'R')
        {
            stopAll();
            lastCommandTime = 0;
        }
    }

    // Print sensor readings every 2 seconds
    if (millis() - lastSensorRead > 2000)
    {
        int sensor1 = analogRead(IR1);
        int sensor2 = analogRead(IR2);
        int sensor3 = analogRead(IR3);
        int sensor4 = analogRead(IR4);
        int sensor5 = analogRead(IR5);

        bool line1 = (sensor1 >= IR_THRESHOLD);
        bool line2 = (sensor2 >= IR_THRESHOLD);
        bool line3 = (sensor3 >= IR_THRESHOLD);
        bool line4 = (sensor4 >= IR_THRESHOLD);
        bool line5 = (sensor5 >= IR_THRESHOLD);

        // Invert sensor readings if LEDs go off on black line
        if (SENSOR_INVERTED)
        {
            line1 = !line1;
            line2 = !line2;
            line3 = !line3;
            line4 = !line4;
            line5 = !line5;
        }

        SerialBT.print("Sensors: ");
        SerialBT.print(line1 ? "1" : "0");
        SerialBT.print(line2 ? "1" : "0");
        SerialBT.print(line3 ? "1" : "0");
        SerialBT.print(line4 ? "1" : "0");
        SerialBT.println(line5 ? "1" : "0");
        SerialBT.println("1=Line, 0=No line (FarR-R-C-L-FarL)");

        lastSensorRead = millis();
    }

    delay(50);
}