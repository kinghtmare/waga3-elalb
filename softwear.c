//*wag3 elAlb#

#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>

// Motor 1 pins
const int MOTOR1_IN1 = 3;  //*GPIO3
const int MOTOR1_IN2 = 23; //*GPIO23
const int MOTOR1_IN3 = 15; //*GPIO15
const int MOTOR1_IN4 = 2;  //*GPIO2
const int MOTOR1_EN1 = 21; //*GPIO21
const int MOTOR1_EN2 = 22; //*GPIO22

// Motor 2 pins
const int MOTOR2_IN1 = 17; //*GPIO17
const int MOTOR2_IN2 = 5;  //*GPIO5
const int MOTOR2_IN3 = 18; //*GPIO18
const int MOTOR2_IN4 = 19; //*GPIO19
const int MOTOR2_EN1 = 4;  //*GPIO4
const int MOTOR2_EN2 = 16; //*GPIO16

// IR sensor pins
const int IR1 = 32; //*GPIO32
const int IR2 = 33; //*GPIO33
const int IR3 = 25; //*GPIO25
const int IR4 = 26; //*GPIO26
const int IR5 = 27; //*GPIO27

// Constants
const int DEFAULT_SPEED = 200;
const int TURN_SPEED = 150;
const int LINE_SPEED = 180;
const int MAX_LINE_LOST_COUNT = 10;
const int IR_THRESHOLD = 2000;    // ADC threshold (0-4095). Tune to your sensors
const int JOYSTICK_DEADZONE = 15; // 0-255 PS4 stick deadzone
const int SERVO_PIN = 14;         // Choose a free PWM-capable pin

Servo panServo;

enum Mode
{
    MANUAL_MODE,
    LINE_FOLLOWING_MODE
};

Mode currentMode = MANUAL_MODE;
unsigned long lastCommandTime = 0;
int lineLostCounter = 0;
bool sensorInverted = false;
unsigned long lastSensorPrint = 0;
unsigned long lastPs4InputTime = 0;

void setupHardware()
{
    // Configure motor control pins as outputs
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

    // Configure IR sensor pins as inputs (analog capable)
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);

    // Servo
    panServo.attach(SERVO_PIN);
    panServo.write(90);
}

void setMotor1Direction(int in1, int in2, int in3, int in4)
{
    digitalWrite(MOTOR1_IN1, in1);
    digitalWrite(MOTOR1_IN2, in2);
    digitalWrite(MOTOR1_IN3, in3);
    digitalWrite(MOTOR1_IN4, in4);
}

void setMotor2Direction(int in1, int in2, int in3, int in4)
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

void moveForward(int speed)
{
    // Motor 1: Forward (to go forward)
    setMotor1Direction(HIGH, LOW, HIGH, LOW);
    setMotor1Speed(speed);

    // Motor 2: Backward (to go forward)
    setMotor2Direction(LOW, HIGH, LOW, HIGH);
    setMotor2Speed(speed);
}

void moveBackward(int speed)
{
    // Motor 1: Backward (to go backward)
    setMotor1Direction(LOW, HIGH, LOW, HIGH);
    setMotor1Speed(speed);

    // Motor 2: Forward (to go backward)
    setMotor2Direction(HIGH, LOW, HIGH, LOW);
    setMotor2Speed(speed);
}

void turnLeft(int speed)
{
    // Motor 1: Backward (right side - to turn left)
    setMotor1Direction(LOW, HIGH, LOW, HIGH);
    setMotor1Speed(speed);

    // Motor 2: Forward (left side - to turn left)
    setMotor2Direction(HIGH, LOW, HIGH, LOW);
    setMotor2Speed(speed);
}

void turnRight(int speed)
{
    // Motor 1: Forward (right side - to turn right)
    setMotor1Direction(HIGH, LOW, HIGH, LOW);
    setMotor1Speed(speed);

    // Motor 2: Backward (left side - to turn right)
    setMotor2Direction(LOW, HIGH, LOW, HIGH);
    setMotor2Speed(speed);
}

void stopMotors()
{
    setMotor1Direction(LOW, LOW, LOW, LOW);
    setMotor2Direction(LOW, LOW, LOW, LOW);
    setMotor1Speed(0);
    setMotor2Speed(0);
}

// line follower
int readSensorDigital(int pin)
{
    int value = digitalRead(pin);
    return sensorInverted ? !value : value;
}

int readSensorAnalog(int pin)
{
    int value = analogRead(pin); // 0-4095 on ESP32
    bool detected = value >= IR_THRESHOLD;
    return sensorInverted ? !detected : detected;
}

bool isLineDetected()
{
    int detectedCount = 0;
    if (readSensorAnalog(IR1) == 1)
        detectedCount++;
    if (readSensorAnalog(IR2) == 1)
        detectedCount++;
    if (readSensorAnalog(IR3) == 1)
        detectedCount++;
    if (readSensorAnalog(IR4) == 1)
        detectedCount++;
    if (readSensorAnalog(IR5) == 1)
        detectedCount++;

    // Debug: More sensitive detection (changed from 2 to 1 for better detection)
    return (detectedCount >= 1);
}

void lineFollow()
{
    int sensor1 = readSensorAnalog(IR1);
    int sensor2 = readSensorAnalog(IR2);
    int sensor3 = readSensorAnalog(IR3);
    int sensor4 = readSensorAnalog(IR4);
    int sensor5 = readSensorAnalog(IR5);

    // Debug: Print sensor readings every 2 seconds
    if (millis() - lastSensorPrint > 2000)
    {
        Serial.print("Sensors: ");
        Serial.print(sensor1);
        Serial.print(sensor2);
        Serial.print(sensor3);
        Serial.print(sensor4);
        Serial.println(sensor5);
        Serial.println("1=Line detected, 0=No line");
        lastSensorPrint = millis();
    }
    // Center sensor on line: go straight
    if (sensor3 == 1 && sensor2 == 0 && sensor4 == 0)
    {
        moveForward(LINE_SPEED);
        lineLostCounter = 0;
    }
    // Left sensors detect line
    else if ((sensor1 == 1 && sensor2 == 1) && sensor4 == 0 && sensor5 == 0)
    {
        turnLeft(LINE_SPEED);
        lineLostCounter = 0;
    }
    // Right sensors detect line
    else if ((sensor4 == 1 && sensor5 == 1) && sensor1 == 0 && sensor2 == 0)
    {
        turnRight(LINE_SPEED);
        lineLostCounter = 0;
    }
    // Cross section detected (both far sensors)
    else if (sensor1 == 1 && sensor5 == 1)
    {
        moveForward(LINE_SPEED);
        delay(300);
        lineLostCounter = 0;
    }
    // Lost line
    else
    {
        lineLostCounter++;
        if (lineLostCounter > MAX_LINE_LOST_COUNT)
        {
            stopMotors();
            Serial.println("Line lost! Switched to Manual Mode");
            currentMode = MANUAL_MODE;
            lineLostCounter = 0;
        }
        else
        {
            moveForward(LINE_SPEED / 2);
            delay(100);
        }
    }
}

// ===== MODE CONTROL FUNCTIONS =====
void toggleMode()
{
    if (currentMode == MANUAL_MODE)
    {
        currentMode = LINE_FOLLOWING_MODE;
        Serial.println("SWITCHED TO LINE FOLLOWING MODE");
        Serial.println("Sensors will show readings every 2 seconds");
    }
    else
    {
        currentMode = MANUAL_MODE;
        Serial.println("SWITCHED TO MANUAL MODE");
        stopMotors();
    }
}

// ===== PS4 MANUAL CONTROL =====
void handlePs4ManualControl()
{
    if (!PS4.isConnected())
    {
        return;
    }

    // Read left stick (-128..127). Convert to -255..255
    int lx = PS4.data.analog.stick.lx; // 0..255 with center ~127
    int ly = PS4.data.analog.stick.ly; // 0..255 with center ~127
    int x = lx - 127;
    int y = 127 - ly; // invert so up is +

    if (abs(x) < JOYSTICK_DEADZONE)
        x = 0;
    if (abs(y) < JOYSTICK_DEADZONE)
        y = 0;

    if (x != 0 || y != 0)
    {
        lastPs4InputTime = millis();
        // Differential mixing
        int left = y + x;
        int right = y - x;

        left = constrain(left, -255, 255);
        right = constrain(right, -255, 255);

        int leftSpeed = map(abs(left), 0, 255, 0, 255);
        int rightSpeed = map(abs(right), 0, 255, 0, 255);
        if (left >= 0 && right >= 0)
        {
            // forward with potential turn
            setMotor1Direction(HIGH, LOW, HIGH, LOW);
            setMotor2Direction(LOW, HIGH, LOW, HIGH);
        }
        else if (left <= 0 && right <= 0)
        {
            // backward
            setMotor1Direction(LOW, HIGH, LOW, HIGH);
            setMotor2Direction(HIGH, LOW, HIGH, LOW);
        }
        else if (left < 0 && right > 0)
        {
            // rotate right
            setMotor1Direction(LOW, HIGH, LOW, HIGH);
            setMotor2Direction(LOW, HIGH, LOW, HIGH);
        }
        else if (left > 0 && right < 0)
        {
            // rotate left
            setMotor1Direction(HIGH, LOW, HIGH, LOW);
            setMotor2Direction(HIGH, LOW, HIGH, LOW);
        }

        setMotor1Speed(leftSpeed);
        setMotor2Speed(rightSpeed);
    }
    else
    {
        // No stick input: stop gradually
        if (millis() - lastPs4InputTime > 300)
        {
            stopMotors();
        }
    }

    // D-Pad quick commands
    if (PS4.Up())
    {
        moveForward(DEFAULT_SPEED);
    }
    else if (PS4.Down())
    {
        moveBackward(DEFAULT_SPEED);
    }
    else if (PS4.Left())
    {
        turnLeft(TURN_SPEED);
    }
    else if (PS4.Right())
    {
        turnRight(TURN_SPEED);
    }

    // Toggle modes with Options button
    if (PS4.Options())
    {
        toggleMode();
        delay(200);
    }

    // Servo quick positions
    if (PS4.event.button_down.cross)
    {
        panServo.write(0);
    }
    if (PS4.event.button_down.triangle)
    {
        panServo.write(90);
    }
    if (PS4.event.button_down.circle)
    {
        panServo.write(180);
    }

    // Invert sensors with Share
    if (PS4.event.button_down.share)
    {
        sensorInverted = !sensorInverted;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("ESP32 Clean Dual Mode Robot Control Starting...");

    setupHardware();
    stopMotors();
    PS4.begin();
    // Start PS4 controller
    PS4.begin();
    Serial.println("PS4 controller initialized. Hold SHARE+PS to pair if needed.");

    Serial.println("Robot Control Ready!");
    Serial.println("Use PS4 controller for manual control. Options button toggles modes.");
}

void loop()
{
    // Manual mode with PS4 controller
    if (currentMode == MANUAL_MODE)
    {
        handlePs4ManualControl();
    }

    // Line following mode
    if (currentMode == LINE_FOLLOWING_MODE)
    {
        if (isLineDetected())
        {
            lineFollow();
        }
        else
        {
            lineLostCounter++;
            if (lineLostCounter > MAX_LINE_LOST_COUNT)
            {
                stopMotors();
                Serial.println("Line lost! Switched to Manual Mode");
                lineLostCounter = 0;
            }
            else
            {
                moveForward(LINE_SPEED / 3);
            }
        }
    }
    delay(50);
}