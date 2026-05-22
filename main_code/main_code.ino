#include <Arduino.h>
#include <WebServer.h>
#include "ble_pid.h"
#include "ota.h"

// ========== PIN CONFIGURATION ==========
const int SENSOR_PINS[] = {27, 26, 25, 33, 32, 35, 39, 36};
const int NUM_SENSORS = 8;

// Motor A (Left)
const int ENA = 19;
const int IN1 = 18;
const int IN2 = 5;

// Motor B (Right)
const int ENB = 23;
const int IN3 = 22;
const int IN4 = 21;

// ========== SENSOR CALIBRATION ==========
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int calibratedValues[NUM_SENSORS];
bool isCalibrated = false;
bool calibrationRequested = false;

// ========== PID & MOTOR VARIABLES ==========
float pidError = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float lastValidPosition = 3.0;

// Moving average filter for position
float positionBuffer[5] = {3.0, 3.0, 3.0, 3.0, 3.0};
int bufferIndex = 0;

// ========== ROBOT STATE MACHINE ==========
enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_STARTING,
    STATE_FOLLOWING,
    STATE_INTERSECTION,
    STATE_STOPPING
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long allBlackStartTime = 0;

WebServer server(80);

// ========== FUNCTION DECLARATIONS ==========
void readSensors();
void autoCalibrate();
float computePosition();
int getLineWidth();
void updateMotorControl(float position);
void setMotorSpeed(int leftSpeed, int rightSpeed);
bool isAllBlack();
float getLineCurvature();
void resetPID();
void emergencyStop();

// ========== SETUP ==========
void setup()
{
    Serial.begin(115200);
    Serial.println("Robot booting...");
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }
    
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    pinMode(2, OUTPUT);
    
    ota_begin(server);
    bleSetup();
    
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    
    Serial.println("Robot ready - with start and intersection handling");
    Serial.println("Press CALIBRATE in app first!");
}

// ========== AUTO CALIBRATION ==========
void autoCalibrate()
{
    Serial.println("======================================");
    Serial.println("AUTO CALIBRATION STARTED");
    Serial.println("Small circle calibration mode");
    Serial.println("======================================");

    digitalWrite(2, HIGH);

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    int leftSpeed = 22;
    int rightSpeed = 42;

    for (int cycle = 0; cycle < 10; cycle++) {

        Serial.print("Circle cycle ");
        Serial.println(cycle + 1);

        setMotorSpeed(leftSpeed, rightSpeed);
        for (int t = 0; t < 90; t++) {
            readSensors();
            for (int i = 0; i < NUM_SENSORS; i++) {
                int val = analogRead(SENSOR_PINS[i]);
                if (val < sensorMin[i]) sensorMin[i] = val;
                if (val > sensorMax[i]) sensorMax[i] = val;
            }
            bleLoop();
            ota_loop(server);
            delay(8);
        }

        setMotorSpeed(rightSpeed, leftSpeed);
        for (int t = 0; t < 90; t++) {
            readSensors();
            for (int i = 0; i < NUM_SENSORS; i++) {
                int val = analogRead(SENSOR_PINS[i]);
                if (val < sensorMin[i]) sensorMin[i] = val;
                if (val > sensorMax[i]) sensorMax[i] = val;
            }
            bleLoop();
            ota_loop(server);
            delay(8);
        }
    }

    setMotorSpeed(0, 0);

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorMax[i] - sensorMin[i] < 100) {
            sensorMax[i] = sensorMin[i] + 1000;
        }
    }

    Serial.println("======================================");
    Serial.println("CALIBRATION COMPLETE!");
    Serial.println("Sensor ranges (min - max):");

    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sensorMin[i]);
        Serial.print(" - ");
        Serial.println(sensorMax[i]);
    }

    Serial.println("======================================");

    isCalibrated = true;
    digitalWrite(2, LOW);
    Serial.println("Ready! Press START to begin race.");
    Serial.println("Place robot on ALL BLACK start square first!");
}

// ========== SENSOR READING ==========
void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(SENSOR_PINS[i]);
        int calibrated = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
        calibrated = constrain(calibrated, 0, 1000);
        calibratedValues[i] = calibrated;
    }
}

int getLineWidth()
{
    int width = 0;
    int threshold = 200;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (calibratedValues[i] > threshold) width++;
    }
    return width;
}

// ========== POSITION CALCULATION ==========
float computePosition()
{
    long weightedSum = 0;
    long totalWeight = 0;
    int threshold = 250;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int weight = (calibratedValues[i] > threshold) ? calibratedValues[i] : 0;
        weightedSum += (long)i * weight;
        totalWeight += weight;
    }

    if (totalWeight == 0) return lastValidPosition;

    float rawPosition = (float)weightedSum / totalWeight;
    rawPosition = constrain(rawPosition, 0.0, 7.0);

    positionBuffer[bufferIndex] = rawPosition;
    bufferIndex = (bufferIndex + 1) % 5;

    float filteredPos = 0;
    for (int i = 0; i < 5; i++) filteredPos += positionBuffer[i];
    filteredPos = filteredPos / 5.0;

    lastValidPosition = filteredPos;
    return filteredPos;
}

// ========== CONDITION DETECTION ==========
bool isAllBlack()
{
    int blackCount = 0;
    int threshold = 350;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (calibratedValues[i] > threshold) blackCount++;
    }
    return (blackCount >= 4);
}

float getLineCurvature()
{
    int leftSum = 0, rightSum = 0;
    int threshold = 250;

    for (int i = 0; i < 3; i++) {
        if (calibratedValues[i] > threshold) leftSum += calibratedValues[i];
    }
    for (int i = 5; i < 8; i++) {
        if (calibratedValues[i] > threshold) rightSum += calibratedValues[i];
    }
    return (float)(rightSum - leftSum) / 100.0;
}

void resetPID()
{
    integral = 0;
    derivative = 0;
    lastError = 0;
}

// ========== MOTOR CONTROL ==========
void updateMotorControl(float position)
{
    float error = 3.5 - position;

    float deviation = abs(error);
    int dynamicBaseSpeed = baseSpeed + 20 - (deviation * 12);
    dynamicBaseSpeed = constrain(dynamicBaseSpeed, 30, maxSpeed);

    int lineWidth = getLineWidth();
    float widthFactor = 1.0;
    if (lineWidth >= 3) widthFactor = 0.9;
    else if (lineWidth <= 1) widthFactor = 1.2;

    float curvature = getLineCurvature();
    float curvatureFactor = 1.0 + (abs(curvature) / 500.0);

    error = error * curvatureFactor * widthFactor;

    integral += error;
    derivative = error - lastError;
    integral = constrain(integral, -200, 200);
    
    pidError = (kp * error) + (ki * integral) + (kd * derivative);
    lastError = error;

    int leftSpeed = dynamicBaseSpeed - pidError;
    int rightSpeed = dynamicBaseSpeed + pidError;

    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

    setMotorSpeed(leftSpeed, rightSpeed);
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftSpeed = -leftSpeed;
    }

    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        rightSpeed = -rightSpeed;
    }

    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    int leftPWM = map(leftSpeed, 0, maxSpeed, 0, 255);
    int rightPWM = map(rightSpeed, 0, maxSpeed, 0, 255);

    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);
}

void emergencyStop()
{
    setMotorSpeed(0, 0);
    currentState = STATE_IDLE;
    resetPID();
    allBlackStartTime = 0;
    digitalWrite(2, LOW);
    Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
}

// ========== STATE HANDLERS ==========
void handleIdle()
{
    setMotorSpeed(0, 0);

    if (calibrationRequested) {
        Serial.println("Calibration requested - starting...");
        currentState = STATE_CALIBRATING;
        calibrationRequested = false;
    }
}

void handleCalibrating()
{
    autoCalibrate();
    currentState = STATE_IDLE;
    calibrationRequested = false;
}

void handleStarting()
{
    readSensors();

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        int blackCount = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (calibratedValues[i] > 400) blackCount++;
        }
        Serial.print("Start mode - Black sensors: ");
        Serial.print(blackCount);
        Serial.print("/8 | Left edge: ");
        Serial.print(calibratedValues[0]);
        Serial.print(" | Right edge: ");
        Serial.println(calibratedValues[7]);
        lastDebug = millis();
    }

    if (isAllBlack()) {
        if (allBlackStartTime == 0) {
            allBlackStartTime = millis();
            Serial.println("On start square - moving forward to find line edge...");
        }
        setMotorSpeed(28, 28);
        
        bool leftEdgeWhite = (calibratedValues[0] < 200);
        bool rightEdgeWhite = (calibratedValues[7] < 200);
        bool centerSeesLine = (calibratedValues[3] > 400 || calibratedValues[4] > 400);

        if (leftEdgeWhite || rightEdgeWhite || centerSeesLine) {
            Serial.println("Start square exit detected!");
            currentState = STATE_FOLLOWING;
            resetPID();
            digitalWrite(2, HIGH);
            setMotorSpeed(0, 0);
            delay(80);
            Serial.println("STARTED! Beginning line follow.");
        }
    } else {
        if (allBlackStartTime != 0) {
            Serial.println("Lost start square - reposition robot");
            allBlackStartTime = 0;
        }
        setMotorSpeed(0, 0);
    }
}

void handleFollowing()
{
    readSensors();

    if (isAllBlack()) {
        if (allBlackStartTime == 0) {
            allBlackStartTime = millis();
        }
        else if (millis() - allBlackStartTime > 300) {
            currentState = STATE_STOPPING;
            stateStartTime = millis();
            Serial.println("END DETECTED - Stopping");
            return;
        }
        else if (millis() - allBlackStartTime > 80) {
            currentState = STATE_INTERSECTION;
            stateStartTime = millis();
            Serial.println("INTERSECTION - Going straight");
            return;
        }
        return;
    }
    allBlackStartTime = 0;

    float position = computePosition();
    updateMotorControl(position);
}

void handleIntersection()
{
    setMotorSpeed(baseSpeed, baseSpeed);
    if (millis() - stateStartTime > 120) {
        currentState = STATE_FOLLOWING;
        resetPID();
        allBlackStartTime = 0;
    }
}

void handleStopping()
{
    setMotorSpeed(0, 0);
    digitalWrite(2, LOW);
    if (millis() - stateStartTime > 5000) {
        currentState = STATE_IDLE;
        allBlackStartTime = 0;
        Serial.println("Ready for next run");
    }
}

// ========== MAIN LOOP ==========
void loop()
{
    ota_loop(server);
    bleLoop();

    switch (currentState) {
        case STATE_IDLE:
            handleIdle();
            break;
        case STATE_CALIBRATING:
            handleCalibrating();
            break;
        case STATE_STARTING:
            handleStarting();
            break;
        case STATE_FOLLOWING:
            handleFollowing();
            break;
        case STATE_INTERSECTION:
            handleIntersection();
            break;
        case STATE_STOPPING:
            handleStopping();
            break;
    }
}

// ========== BLE CALLBACKS ==========
void onCalibrateRequest()
{
    calibrationRequested = true;
    Serial.println("Calibration requested from app");
}

void onStartRequest()
{
    Serial.print("START requested. Calibrated: ");
    Serial.println(isCalibrated ? "YES" : "NO");

    if (isCalibrated && currentState == STATE_IDLE) {
        currentState = STATE_STARTING;
        allBlackStartTime = 0;
        Serial.println("Entering STARTING state - waiting for start square");
        Serial.println("Place robot on ALL BLACK square!");
    }
    else if (!isCalibrated) {
        Serial.println("Cannot start - calibrate first! Press CALIBRATE button.");
    }
    else {
        Serial.print("Cannot start - current state: ");
        Serial.println(currentState);
    }
}

void onStopRequest()
{
    emergencyStop();
}

void onSpeedUpdate(int newBaseSpeed, int newMaxSpeed)
{
    baseSpeed = constrain(newBaseSpeed, 20, 100);
    maxSpeed = constrain(newMaxSpeed, 40, 150);
    Serial.printf("Speed updated: Base=%d, Max=%d\n", baseSpeed, maxSpeed);
}