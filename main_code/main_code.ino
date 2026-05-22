#include <Arduino.h>
#include <WebServer.h>
#include "ble_pid.h"
#include "ota.h"

// Pin configuration
const int SENSOR_PINS[] = {27, 26, 25, 33, 32, 35, 39, 36};
const int NUM_SENSORS = 8;

// Left motor
const int ENA = 19;
const int IN1 = 18;
const int IN2 = 5;

// Right motor
const int ENB = 23;
const int IN3 = 22;
const int IN4 = 21;

// Sensor calibration
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int calibratedValues[NUM_SENSORS];
bool isCalibrated = false;
bool calibrationRequested = false;

// PID variables
float pidError = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float lastValidPosition = 3.5;
int lastValidSide = 0;

float positionBuffer[3] = {3.5, 3.5, 3.5};
int bufferIndex = 0;

// Line loss recovery
bool lineLost = false;
unsigned long lineLostTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 250;

// Lookahead detection
bool lookaheadActive = false;
const unsigned long LOOKAHEAD_DRIVE_MS = 80;
const int LOOKAHEAD_SPEED = 28;

// Robot states
enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_STARTING,
    STATE_FOLLOWING,
    STATE_LOOKAHEAD,
    STATE_FAKE_BRANCH,
    STATE_TURN_MODE,
    STATE_STEP_RECOVERY,
    STATE_SPIRAL_MODE,
    STATE_INTERSECTION,
    STATE_STOPPING
};

RobotState currentState = STATE_IDLE;

unsigned long stateStartTime = 0;
unsigned long allBlackStartTime = 0;

WebServer server(80);

// Function declarations
void readSensors();
void autoCalibrate();
float computePosition(bool &outLineLost);
int getLineWidth();
void updateMotorControl(float position);
void setMotorSpeed(int leftSpeed, int rightSpeed);
bool isAllBlack();
bool edgeSpikeDetected();
bool centerSeesLine();
float getLineCurvature();
void resetPID();
void emergencyStop();

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

    Serial.println("Robot ready");
}

void autoCalibrate()
{
    Serial.println("Calibration started");
    digitalWrite(2, HIGH);

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    int leftSpeed = 22;
    int rightSpeed = 42;

    for (int cycle = 0; cycle < 10; cycle++) {

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

    isCalibrated = true;

    digitalWrite(2, LOW);

    Serial.println("Calibration complete");
}

void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {

        int raw = analogRead(SENSOR_PINS[i]);

        calibratedValues[i] = constrain(
            map(raw, sensorMin[i], sensorMax[i], 0, 1000),
            0,
            1000
        );
    }
}

int getLineWidth()
{
    int w = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {

        if (calibratedValues[i] > 200) {
            w++;
        }
    }

    return w;
}

float computePosition(bool &outLineLost)
{
    long weightedSum = 0;
    long totalWeight = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {

        int weight = (calibratedValues[i] > 250)
                     ? calibratedValues[i]
                     : 0;

        weightedSum += (long)i * weight;
        totalWeight += weight;
    }

    if (totalWeight == 0) {

        outLineLost = true;

        return (lastValidPosition < 3.5f)
               ? 0.0f
               : 7.0f;
    }

    outLineLost = false;

    float rawPos = constrain(
        (float)weightedSum / totalWeight,
        0.0f,
        7.0f
    );

    if (rawPos < 3.0f) {
        lastValidSide = -1;
    }
    else if (rawPos > 4.0f) {
        lastValidSide = 1;
    }
    else {
        lastValidSide = 0;
    }

    positionBuffer[bufferIndex] = rawPos;

    bufferIndex = (bufferIndex + 1) % 3;

    float filtered = 0;

    for (int i = 0; i < 3; i++) {
        filtered += positionBuffer[i];
    }

    filtered /= 3.0f;

    lastValidPosition = filtered;

    return filtered;
}

bool isAllBlack()
{
    int c = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {

        if (calibratedValues[i] > 350) {
            c++;
        }
    }

    return (c >= 4);
}

bool edgeSpikeDetected()
{
    int leftCount = 0;
    int rightCount = 0;
    int centerCount = 0;

    for (int i = 0; i <= 2; i++) {
        if (calibratedValues[i] > 350) leftCount++;
    }

    for (int i = 5; i <= 7; i++) {
        if (calibratedValues[i] > 350) rightCount++;
    }

    for (int i = 3; i <= 4; i++) {
        if (calibratedValues[i] > 300) centerCount++;
    }

    bool sideSpike = (leftCount >= 2 || rightCount >= 2);
    bool centerActive = (centerCount >= 1);

    return (sideSpike && centerActive);
}

bool centerSeesLine()
{
    return (
        calibratedValues[3] > 300 ||
        calibratedValues[4] > 300
    );
}

float getLineCurvature()
{
    int leftSum = 0;
    int rightSum = 0;

    for (int i = 0; i < 3; i++) {

        if (calibratedValues[i] > 250) {
            leftSum += calibratedValues[i];
        }
    }

    for (int i = 5; i < 8; i++) {

        if (calibratedValues[i] > 250) {
            rightSum += calibratedValues[i];
        }
    }

    return (float)(rightSum - leftSum) / 100.0f;
}

void resetPID()
{
    integral = 0;
    derivative = 0;
    lastError = 0;
}

void updateMotorControl(float position)
{
    float error = 3.5f - position;

    float deviation = abs(error);

    int dynBase = constrain(
        baseSpeed + 20 - (int)(deviation * 18),
        25,
        maxSpeed
    );

    int lw = getLineWidth();

    float wf =
        (lw >= 3) ? 0.9f :
        (lw <= 1) ? 1.2f :
        1.0f;

    float cf =
        1.0f + (abs(getLineCurvature()) / 500.0f);

    error = error * cf * wf;

    integral += error;

    derivative = error - lastError;

    integral = constrain(integral, -100, 100);

    pidError =
        (kp * error) +
        (ki * integral) +
        (kd * derivative);

    lastError = error;

    int ls = constrain(
        dynBase - (int)pidError,
        -maxSpeed,
        maxSpeed
    );

    int rs = constrain(
        dynBase + (int)pidError,
        -maxSpeed,
        maxSpeed
    );

    setMotorSpeed(ls, rs);
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    if (leftSpeed >= 0) {

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        leftSpeed = -leftSpeed;
    }

    if (rightSpeed >= 0) {

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    else {

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        rightSpeed = -rightSpeed;
    }

    analogWrite(
        ENA,
        map(constrain(leftSpeed, 0, maxSpeed), 0, maxSpeed, 0, 255)
    );

    analogWrite(
        ENB,
        map(constrain(rightSpeed, 0, maxSpeed), 0, maxSpeed, 0, 255)
    );
}

void emergencyStop()
{
    setMotorSpeed(0, 0);

    currentState = STATE_IDLE;

    lineLost = false;
    lineLostTime = 0;

    lookaheadActive = false;

    resetPID();

    allBlackStartTime = 0;

    digitalWrite(2, LOW);

    Serial.println("Emergency stop");
}