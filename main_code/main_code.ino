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

int  sensorMin[NUM_SENSORS];
int  sensorMax[NUM_SENSORS];
int  calibratedValues[NUM_SENSORS];
bool isCalibrated         = false;
bool calibrationRequested = false;

float pidError          = 0;
float lastError         = 0;
float integral          = 0;
float derivative        = 0;
float lastValidPosition = 3.5f;
int   lastValidSide     = 0;

float positionBuffer[3] = {3.5f, 3.5f, 3.5f};
int   bufferIndex       = 0;

bool          lineLost     = false;
unsigned long lineLostTime = 0;

const unsigned long LINE_LOST_TIMEOUT = 180;
const unsigned long SPIN_CREEP_MS     = 35;
const int           SPIN_CREEP_SPEED  = 18;
const int           SPIN_START_SPEED  = 20;
const int           SPIN_RAMP_MS      = 90;

unsigned long lastEdgeSeenTime = 0;
int           lastEdgeSide     = 0;
const unsigned long EDGE_MEMORY_MS  = 200;
const int           ARC_SPEED       = 22;
const int           ARC_OUTER_BOOST = 7;

const int EDGE_STRONG_THRESHOLD = 400;
const int CORNER_APPROACH_SPEED = 28;

const int           SNAP_TURN_SPEED     = 28;
const unsigned long SNAP_TURN_MAX_MS    = 400;
const unsigned long SNAP_TURN_SETTLE_MS = 60;
const int           FAKE_EDGE_THRESHOLD = 350;
const int           LINE_FOUND_THRESH   = 300;

int           snapTurnDirection = 0;
unsigned long snapTurnStartTime = 0;
unsigned long snapTurnDuration  = 0;

enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_STARTING,
    STATE_FOLLOWING,
    STATE_SNAP_TURN,
    STATE_UNDO_TURN,
    STATE_INTERSECTION,
    STATE_STOPPING
};

RobotState    currentState      = STATE_IDLE;
unsigned long stateStartTime    = 0;
unsigned long allBlackStartTime = 0;

WebServer server(80);

void  readSensors();
void  autoCalibrate();
float computePosition(bool &outLineLost);
int   getLineWidth();
void  updateMotorControl(float position);
void  setMotorSpeed(int leftSpeed, int rightSpeed);
bool  isAllBlack();
bool  centerSeesLine();
float getLineCurvature();
void  resetPID();
void  emergencyStop();
void  updateEdgeMemory();

void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(2,   OUTPUT);

    ota_begin(server);
    bleSetup();

    digitalWrite(2, HIGH); delay(500); digitalWrite(2, LOW);
}

void autoCalibrate()
{
    digitalWrite(2, HIGH);

    for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

    const int SPIN_CAL_SPEED = 32;

    for (int cycle = 0; cycle < 10; cycle++) {
        setMotorSpeed(SPIN_CAL_SPEED, -SPIN_CAL_SPEED);
        for (int t = 0; t < 90; t++) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                int val = analogRead(SENSOR_PINS[i]);
                if (val < sensorMin[i]) sensorMin[i] = val;
                if (val > sensorMax[i]) sensorMax[i] = val;
            }
            bleLoop(); ota_loop(server); delay(8);
        }

        setMotorSpeed(-SPIN_CAL_SPEED, SPIN_CAL_SPEED);
        for (int t = 0; t < 90; t++) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                int val = analogRead(SENSOR_PINS[i]);
                if (val < sensorMin[i]) sensorMin[i] = val;
                if (val > sensorMax[i]) sensorMax[i] = val;
            }
            bleLoop(); ota_loop(server); delay(8);
        }
    }

    setMotorSpeed(0, 0);

    for (int i = 0; i < NUM_SENSORS; i++)
        if (sensorMax[i] - sensorMin[i] < 100) sensorMax[i] = sensorMin[i] + 1000;

    isCalibrated = true;
    digitalWrite(2, LOW);
}

void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(SENSOR_PINS[i]);
        calibratedValues[i] = constrain(
            map(raw, sensorMin[i], sensorMax[i], 0, 1000), 0, 1000);
    }
}

int getLineWidth()
{
    int w = 0;
    for (int i = 0; i < NUM_SENSORS; i++) if (calibratedValues[i] > 250) w++;
    return w;
}

void updateEdgeMemory()
{
    if (calibratedValues[0] > 350) { lastEdgeSeenTime = millis(); lastEdgeSide = -1; }
    if (calibratedValues[7] > 350) { lastEdgeSeenTime = millis(); lastEdgeSide =  1; }
}

float computePosition(bool &outLineLost)
{
    long weightedSum = 0, totalWeight = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int weight = (calibratedValues[i] > 250) ? calibratedValues[i] : 0;
        weightedSum += (long)i * weight;
        totalWeight += weight;
    }

    if (totalWeight == 0) {
        outLineLost = true;
        return (lastValidPosition < 3.5f) ? 0.0f : 7.0f;
    }

    outLineLost = false;
    float rawPos = constrain((float)weightedSum / totalWeight, 0.0f, 7.0f);

    if      (rawPos < 3.0f) lastValidSide = -1;
    else if (rawPos > 4.0f) lastValidSide =  1;
    else                    lastValidSide =  0;

    positionBuffer[bufferIndex] = rawPos;
    bufferIndex = (bufferIndex + 1) % 3;

    float filtered = 0;
    for (int i = 0; i < 3; i++) filtered += positionBuffer[i];
    filtered /= 3.0f;

    lastValidPosition = filtered;
    return filtered;
}

bool isAllBlack()
{
    int c = 0;
    for (int i = 0; i < NUM_SENSORS; i++) if (calibratedValues[i] > 350) c++;
    return (c >= 4);
}

bool centerSeesLine()
{
    return (calibratedValues[2] > LINE_FOUND_THRESH ||
            calibratedValues[3] > LINE_FOUND_THRESH ||
            calibratedValues[4] > LINE_FOUND_THRESH ||
            calibratedValues[5] > LINE_FOUND_THRESH);
}

float getLineCurvature()
{
    int leftSum = 0, rightSum = 0;
    for (int i = 0; i < 3; i++) if (calibratedValues[i] > 250) leftSum  += calibratedValues[i];
    for (int i = 5; i < 8; i++) if (calibratedValues[i] > 250) rightSum += calibratedValues[i];
    return (float)(rightSum - leftSum) / 100.0f;
}

void resetPID()
{
    integral   = 0;
    derivative = 0;
    lastError  = 0;

    for (int i = 0; i < 3; i++) positionBuffer[i] = lastValidPosition;
    bufferIndex      = 0;
    lastEdgeSeenTime = 0;
}

void updateMotorControl(float position)
{
    float error     = 3.5f - position;
    float deviation = abs(error);
    int   lw        = getLineWidth();

    bool narrowLine      = (lw <= 2);
    bool leftEdgeStrong  = (calibratedValues[0] >= EDGE_STRONG_THRESHOLD);
    bool rightEdgeStrong = (calibratedValues[7] >= EDGE_STRONG_THRESHOLD);

    bool cornerApproach = narrowLine &&
                          ((leftEdgeStrong  && lastValidSide <= 0) ||
                           (rightEdgeStrong && lastValidSide >= 0));

    int dynBase;
    if (cornerApproach) {
        dynBase  = CORNER_APPROACH_SPEED;
        integral = 0;
    } else {
        dynBase = constrain(baseSpeed + 8 - (int)(deviation * 18), 25, maxSpeed);
    }

    float wf = (lw >= 3) ? 0.9f : (lw <= 1) ? 1.2f : 1.0f;
    float cf = 1.0f + (abs(getLineCurvature()) / 500.0f);
    error = error * cf * wf;

    integral  += error;
    derivative = error - lastError;
    integral   = constrain(integral, -100, 100);
    pidError   = (kp * error) + (ki * integral) + (kd * derivative);
    lastError  = error;

    int ls = constrain(dynBase - (int)pidError, -maxSpeed, maxSpeed);
    int rs = constrain(dynBase + (int)pidError, -maxSpeed, maxSpeed);
    setMotorSpeed(ls, rs);
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    if (leftSpeed  >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
    else                 { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); leftSpeed  = -leftSpeed;  }

    if (rightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
    else                 { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); rightSpeed = -rightSpeed; }

    analogWrite(ENA, map(constrain(leftSpeed,  0, maxSpeed), 0, maxSpeed, 0, 255));
    analogWrite(ENB, map(constrain(rightSpeed, 0, maxSpeed), 0, maxSpeed, 0, 255));
}

void emergencyStop()
{
    setMotorSpeed(0, 0);
    currentState      = STATE_IDLE;
    lineLost          = false;
    lineLostTime      = 0;
    resetPID();
    allBlackStartTime = 0;
    digitalWrite(2, LOW);
}

void handleIdle()
{
    setMotorSpeed(0, 0);
    if (calibrationRequested) {
        currentState         = STATE_CALIBRATING;
        calibrationRequested = false;
    }
}

void handleCalibrating()
{
    autoCalibrate();
    currentState         = STATE_IDLE;
    calibrationRequested = false;
}

void handleStarting()
{
    readSensors();

    if (isAllBlack()) {
        if (allBlackStartTime == 0) allBlackStartTime = millis();
        setMotorSpeed(28, 28);
        if (calibratedValues[0] < 200 || calibratedValues[7] < 200 ||
            calibratedValues[3] > 400 || calibratedValues[4] > 400) {
            currentState = STATE_FOLLOWING;
            resetPID();
            digitalWrite(2, HIGH);
            setMotorSpeed(0, 0);
            delay(80);
        }
    } else {
        allBlackStartTime = 0;
        setMotorSpeed(0, 0);
    }
}

void handleFollowing()
{
    readSensors();
    updateEdgeMemory();

    if (isAllBlack()) {
        if (allBlackStartTime == 0) allBlackStartTime = millis();
        if (millis() - allBlackStartTime > 300) {
            currentState   = STATE_STOPPING;
            stateStartTime = millis();
            return;
        }
        if (millis() - allBlackStartTime > 80) {
            currentState   = STATE_INTERSECTION;
            stateStartTime = millis();
            return;
        }
        return;
    }
    allBlackStartTime = 0;

    bool leftEdgeFired  = (calibratedValues[0] > EDGE_STRONG_THRESHOLD);
    bool rightEdgeFired = (calibratedValues[7] > EDGE_STRONG_THRESHOLD);
    bool narrowLine     = (getLineWidth() <= 3);

    if (narrowLine && (leftEdgeFired || rightEdgeFired)) {
        if (leftEdgeFired && rightEdgeFired)
            snapTurnDirection = (lastValidSide <= 0) ? -1 : 1;
        else
            snapTurnDirection = leftEdgeFired ? -1 : 1;

        snapTurnStartTime = millis();
        currentState      = STATE_SNAP_TURN;
        return;
    }

    bool  lost     = false;
    float position = computePosition(lost);

    if (lost) {
        unsigned long now     = millis();
        unsigned long elapsed = now - lineLostTime;

        if (!lineLost) {
            lineLost     = true;
            lineLostTime = now;
            elapsed      = 0;
        }

        if (elapsed > LINE_LOST_TIMEOUT) {
            setMotorSpeed(0, 0);
            lineLost = false;
            return;
        }

        bool edgeRecent = (millis() - lastEdgeSeenTime) < EDGE_MEMORY_MS;

        if (edgeRecent) {
            if (lastEdgeSide < 0) setMotorSpeed(ARC_SPEED, ARC_SPEED + ARC_OUTER_BOOST);
            else                  setMotorSpeed(ARC_SPEED + ARC_OUTER_BOOST, ARC_SPEED);
        } else {
            if (elapsed < SPIN_CREEP_MS) {
                setMotorSpeed(SPIN_CREEP_SPEED, SPIN_CREEP_SPEED);
            } else {
                float ramp = constrain(
                    (float)(elapsed - SPIN_CREEP_MS) / SPIN_RAMP_MS, 0.0f, 1.0f);
                int sp = (int)(SPIN_START_SPEED + ramp * ((baseSpeed + 10) - SPIN_START_SPEED));
                if (lastValidSide <= 0) setMotorSpeed(-sp,  sp);
                else                    setMotorSpeed( sp, -sp);
            }
        }
        return;
    }

    if (lineLost) {
        lineLost = false;
        resetPID();
    }

    updateMotorControl(position);
}

void handleSnapTurn()
{
    readSensors();
    unsigned long elapsed = millis() - snapTurnStartTime;

    if (snapTurnDirection < 0) setMotorSpeed(-SNAP_TURN_SPEED,  SNAP_TURN_SPEED);
    else                       setMotorSpeed( SNAP_TURN_SPEED, -SNAP_TURN_SPEED);

    if (elapsed > SNAP_TURN_SETTLE_MS) {
        bool oppositeSees = (snapTurnDirection < 0)
                            ? (calibratedValues[7] > FAKE_EDGE_THRESHOLD)
                            : (calibratedValues[0] > FAKE_EDGE_THRESHOLD);

        if (oppositeSees) {
            snapTurnDuration = elapsed;
            currentState     = STATE_UNDO_TURN;
            stateStartTime   = millis();
            return;
        }

        if (centerSeesLine()) {
            lineLost     = false;
            currentState = STATE_FOLLOWING;
            resetPID();
            return;
        }
    }

    if (elapsed > SNAP_TURN_MAX_MS) {
        lineLost     = true;
        lineLostTime = millis();
        currentState = STATE_FOLLOWING;
        resetPID();
    }
}

void handleUndoTurn()
{
    unsigned long elapsed = millis() - stateStartTime;

    if (snapTurnDirection < 0) setMotorSpeed( SNAP_TURN_SPEED, -SNAP_TURN_SPEED);
    else                       setMotorSpeed(-SNAP_TURN_SPEED,  SNAP_TURN_SPEED);

    if (elapsed >= snapTurnDuration) {
        setMotorSpeed(0, 0);
        delay(30);
        currentState = STATE_FOLLOWING;
        resetPID();
    }
}

void handleIntersection()
{
    setMotorSpeed(baseSpeed, baseSpeed);
    if (millis() - stateStartTime > 120) {
        currentState      = STATE_FOLLOWING;
        allBlackStartTime = 0;
        resetPID();
    }
}

void handleStopping()
{
    setMotorSpeed(0, 0);
    digitalWrite(2, LOW);
    if (millis() - stateStartTime > 5000) {
        currentState      = STATE_IDLE;
        allBlackStartTime = 0;
    }
}

void loop()
{
    ota_loop(server);
    bleLoop();

    switch (currentState) {
        case STATE_IDLE:         handleIdle();          break;
        case STATE_CALIBRATING:  handleCalibrating();   break;
        case STATE_STARTING:     handleStarting();      break;
        case STATE_FOLLOWING:    handleFollowing();     break;
        case STATE_SNAP_TURN:    handleSnapTurn();      break;
        case STATE_UNDO_TURN:    handleUndoTurn();      break;
        case STATE_INTERSECTION: handleIntersection();  break;
        case STATE_STOPPING:     handleStopping();      break;
    }
}

void onCalibrateRequest() { calibrationRequested = true; }

void onStartRequest()
{
    if (isCalibrated && currentState == STATE_IDLE) {
        currentState      = STATE_STARTING;
        allBlackStartTime = 0;
    }
}

void onStopRequest() { emergencyStop(); }

void onSpeedUpdate(int newBaseSpeed, int newMaxSpeed)
{
    baseSpeed = constrain(newBaseSpeed, 20, 100);
    maxSpeed  = constrain(newMaxSpeed,  40, 150);
}