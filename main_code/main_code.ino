#include <Arduino.h>
#include <WebServer.h>
#include "ble_pid.h"
#include "ota.h"

const int SENSOR_PINS[] = {27, 26, 25, 33, 32, 35, 39, 36};
const int NUM_SENSORS = 8;

const int ENA = 19;
const int IN1 = 18;
const int IN2 = 5;
const int ENB = 23;
const int IN3 = 22;
const int IN4 = 21;

const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int CH_A = 0;
const int CH_B = 1;

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int calibratedValues[NUM_SENSORS];
bool isCalibrated = false;
bool calibrationRequested = false;

enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING
};

RobotState currentState = STATE_IDLE;

WebServer server(80);

void readSensors();
void autoCalibrate();
void setMotorSpeed(int leftSpeed, int rightSpeed);

void setup()
{
    Serial.begin(115200);
    
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
    
    ledcSetup(CH_A, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, CH_A);
    ledcSetup(CH_B, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENB, CH_B);
    
    pinMode(2, OUTPUT);
    
    ota_begin(server);
    bleSetup();
    
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
}

void autoCalibrate()
{
    Serial.println("AUTO CALIBRATION STARTED");
    digitalWrite(2, HIGH);
    
    for (int cycle = 0; cycle < 3; cycle++) {
        setMotorSpeed(30, 30);
        for (int t = 0; t < 40; t++) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                int val = analogRead(SENSOR_PINS[i]);
                if (val < sensorMin[i]) sensorMin[i] = val;
                if (val > sensorMax[i]) sensorMax[i] = val;
            }
            delay(5);
        }
        
        setMotorSpeed(-30, -30);
        for (int t = 0; t < 40; t++) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                int val = analogRead(SENSOR_PINS[i]);
                if (val < sensorMin[i]) sensorMin[i] = val;
                if (val > sensorMax[i]) sensorMax[i] = val;
            }
            delay(5);
        }
    }
    
    setMotorSpeed(0, 0);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorMax[i] - sensorMin[i] < 100) {
            sensorMax[i] = sensorMin[i] + 1000;
        }
    }
    
    isCalibrated = true;
    digitalWrite(2, LOW);
    Serial.println("Calibration done.");
}

void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(SENSOR_PINS[i]);
        int calibrated = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
        calibrated = constrain(calibrated, 0, 1000);
        calibratedValues[i] = calibrated;
    }
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(CH_A, map(leftSpeed, 0, 100, 0, 255));
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(CH_A, map(-leftSpeed, 0, 100, 0, 255));
    }
    
    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(CH_B, map(rightSpeed, 0, 100, 0, 255));
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(CH_B, map(-rightSpeed, 0, 100, 0, 255));
    }
}

void handleIdle()
{
    setMotorSpeed(0, 0);
    if (calibrationRequested) {
        currentState = STATE_CALIBRATING;
        calibrationRequested = false;
    }
}

void handleCalibrating()
{
    autoCalibrate();
    currentState = STATE_IDLE;
}

void loop()
{
    ota_loop(server);
    bleLoop();
    
    switch (currentState) {
        case STATE_IDLE: handleIdle(); break;
        case STATE_CALIBRATING: handleCalibrating(); break;
    }
}

void onCalibrateRequest()
{
    calibrationRequested = true;
    Serial.println("Calibration requested");
}

void onStartRequest()
{
    Serial.print("START. Cal:"); 
    Serial.println(isCalibrated ? "YES" : "NO");
    if (isCalibrated && currentState == STATE_IDLE) {
        Serial.println("STARTING - but need to implement starting state");
    } else if (!isCalibrated) {
        Serial.println("Calibrate first!");
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
    Serial.printf("Speed: Base=%d Max=%d\n", baseSpeed, maxSpeed);
}