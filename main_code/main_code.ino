#include <Arduino.h>
#include <WebServer.h>
#include "ble_pid.h"
#include "ota.h"

// PIN CONFIGURATION
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

// SENSOR CALIBRATION 
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int calibratedValues[NUM_SENSORS];
bool isCalibrated = false;
bool calibrationRequested = false;

//  ROBOT STATE MACHINE 
enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING
};

RobotState currentState = STATE_IDLE;

WebServer server(80);

// FUNCTION DECLARATIONS 
void readSensors();
void autoCalibrate();
void setMotorSpeed(int leftSpeed, int rightSpeed);

//  SETUP 
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
    
    Serial.println("Robot ready - Press CALIBRATE in app first!");
}

//  AUTO CALIBRATION 
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

        // Clockwise small circle
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

        // Counter-clockwise small circle
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
}

// SENSOR READING 
void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {

        int raw = analogRead(SENSOR_PINS[i]);

        int calibrated = map(raw, sensorMin[i], sensorMax[i], 0, 1000);

        calibrated = constrain(calibrated, 0, 1000);

        calibratedValues[i] = calibrated;
    }
}

// MOTOR CONTROL 
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

// STATE HANDLERS 
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

//  MAIN LOOP 
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
    }
}

// BLE CALLBACKS 
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
        Serial.println("Ready to start - will implement starting state next");
    }
    else if (!isCalibrated) {
        Serial.println("Cannot start - calibrate first! Press CALIBRATE button.");
    }
}

void onStopRequest()
{
    setMotorSpeed(0, 0);
    Serial.println("EMERGENCY STOP");
}

void onSpeedUpdate(int newBaseSpeed, int newMaxSpeed)
{
    baseSpeed = constrain(newBaseSpeed, 20, 100);
    maxSpeed = constrain(newMaxSpeed, 40, 150);
    Serial.printf("Speed updated: Base=%d, Max=%d\n", baseSpeed, maxSpeed);
}