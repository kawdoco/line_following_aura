#include <Arduino.h>
#include <WebServer.h>
#include "ble_pid.h"
#include "ota.h"

// ========== PIN CONFIGURATION ==========
const int SENSOR_PINS[] = {27, 26, 25, 33, 32, 35, 39, 36};
const int NUM_SENSORS = 8;

const int ENA = 19;
const int IN1 = 18;
const int IN2 = 5;

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
float pidError      = 0;
float lastError     = 0;
float integral      = 0;
float derivative    = 0;
float lastValidPosition = 3.5f;
int   lastValidSide = 0;

float positionBuffer[3] = {3.5f, 3.5f, 3.5f};
int   bufferIndex = 0;

bool lineLost = false;
unsigned long lineLostTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 150;

const int  SPIN_START_SPEED = 22;
const int  SPIN_RAMP_MS     = 80;

const unsigned long SPIN_CREEP_MS    = 40;
const int           SPIN_CREEP_SPEED = 20;

bool lookaheadActive = false;
const unsigned long LOOKAHEAD_DRIVE_MS = 80;
const int           LOOKAHEAD_SPEED    = 28;

// ========== ROBOT STATE MACHINE ==========
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

RobotState currentState   = STATE_IDLE;
unsigned long stateStartTime    = 0;
unsigned long allBlackStartTime = 0;

WebServer server(80);

// ========== FUNCTION DECLARATIONS ==========
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

    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(2, OUTPUT);

    ota_begin(server);
    bleSetup();

    digitalWrite(2, HIGH); delay(500); digitalWrite(2, LOW);

    Serial.println("Robot ready. Press CALIBRATE first!");
}

// ========== AUTO CALIBRATION ==========
void autoCalibrate()
{
    Serial.println("=== CALIBRATION STARTED (in-place spin) ===");
    digitalWrite(2, HIGH);

    for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

    const int SPIN_CAL_SPEED = 32;

    for (int cycle = 0; cycle < 10; cycle++) {
        Serial.print("Cycle "); Serial.println(cycle + 1);

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

    Serial.println("=== CALIBRATION COMPLETE ===");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("S"); Serial.print(i); Serial.print(": ");
        Serial.print(sensorMin[i]); Serial.print(" - "); Serial.println(sensorMax[i]);
    }

    isCalibrated = true;
    digitalWrite(2, LOW);
    Serial.println("Ready! Place robot on ALL BLACK square then press START.");
}

// ========== SENSOR READING ==========
void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(SENSOR_PINS[i]);
        calibratedValues[i] = constrain(map(raw, sensorMin[i], sensorMax[i], 0, 1000), 0, 1000);
    }
}

int getLineWidth()
{
    int w = 0;
    for (int i = 0; i < NUM_SENSORS; i++) if (calibratedValues[i] > 200) w++;
    return w;
}

// ========== POSITION CALCULATION ==========
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
    else                     lastValidSide =  0;

    positionBuffer[bufferIndex] = rawPos;
    bufferIndex = (bufferIndex + 1) % 3;

    float filtered = 0;
    for (int i = 0; i < 3; i++) filtered += positionBuffer[i];
    filtered /= 3.0f;

    lastValidPosition = filtered;
    return filtered;
}

// ========== DETECTION HELPERS ==========
bool isAllBlack()
{
    int c = 0;
    for (int i = 0; i < NUM_SENSORS; i++) if (calibratedValues[i] > 350) c++;
    return (c >= 4);
}

bool edgeSpikeDetected()
{
    int leftCount  = 0, rightCount  = 0;
    int centerCount = 0, totalActive = 0;

    for (int i = 0; i < NUM_SENSORS; i++) if (calibratedValues[i] > 300) totalActive++;

    for (int i = 0; i <= 2; i++) if (calibratedValues[i] > 350) leftCount++;
    for (int i = 5; i <= 7; i++) if (calibratedValues[i] > 350) rightCount++;
    for (int i = 2; i <= 5; i++) if (calibratedValues[i] > 250) centerCount++;

    bool sideSpike    = (leftCount >= 2 || rightCount >= 2);
    bool centerActive = (centerCount >= 1);
    bool wideLine     = (totalActive >= 3);

    return (sideSpike && centerActive && wideLine);
}

bool centerSeesLine()
{
    return (calibratedValues[2] > 250 ||
            calibratedValues[3] > 250 ||
            calibratedValues[4] > 250 ||
            calibratedValues[5] > 250);
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
    integral = 0; derivative = 0; lastError = 0;
}

// ========== MOTOR CONTROL ==========
void updateMotorControl(float position)
{
    float error     = 3.5f - position;
    float deviation = abs(error);

    int dynBase = constrain(baseSpeed + 8 - (int)(deviation * 18), 25, maxSpeed);

    int   lw = getLineWidth();
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
    lookaheadActive   = false;
    resetPID();
    allBlackStartTime = 0;
    digitalWrite(2, LOW);
    Serial.println("!!! EMERGENCY STOP !!!");
}

// ========== STATE HANDLERS ==========
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
    calibrationRequested = false;
}

void handleStarting()
{
    readSensors();

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        int bc = 0;
        for (int i = 0; i < NUM_SENSORS; i++) if (calibratedValues[i] > 400) bc++;
        Serial.print("Start black:"); Serial.print(bc);
        Serial.print(" L:"); Serial.print(calibratedValues[0]);
        Serial.print(" R:"); Serial.println(calibratedValues[7]);
        lastDebug = millis();
    }

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
            Serial.println("STARTED!");
        }
    } else {
        allBlackStartTime = 0;
        setMotorSpeed(0, 0);
    }
}

void handleFollowing()
{
    readSensors();

    if (isAllBlack()) {
        if (allBlackStartTime == 0) allBlackStartTime = millis();
        if (millis() - allBlackStartTime > 300) {
            currentState = STATE_STOPPING; stateStartTime = millis();
            Serial.println("END"); return;
        }
        if (millis() - allBlackStartTime > 80) {
            currentState = STATE_INTERSECTION; stateStartTime = millis();
            Serial.println("INTERSECTION"); return;
        }
        return;
    }
    allBlackStartTime = 0;

    if (edgeSpikeDetected()) {
        Serial.println("Edge spike! Lookahead check...");
        currentState   = STATE_LOOKAHEAD;
        stateStartTime = millis();
        setMotorSpeed(LOOKAHEAD_SPEED, LOOKAHEAD_SPEED);
        return;
    }

    bool lost = false;
    float position = computePosition(lost);

    if (lost) {
        unsigned long now = millis();

        if (!lineLost) {
            lineLost     = true;
            lineLostTime = now;
            Serial.print("Line lost - creep then spin ");
            Serial.println(lastValidSide <= 0 ? "LEFT" : "RIGHT");
        }

        unsigned long elapsed = now - lineLostTime;

        if (elapsed > LINE_LOST_TIMEOUT) {
            setMotorSpeed(0, 0);
            lineLost = false;
            return;
        }

        if (elapsed < SPIN_CREEP_MS) {
            setMotorSpeed(SPIN_CREEP_SPEED, SPIN_CREEP_SPEED);
        } else {
            unsigned long spinElapsed = elapsed - SPIN_CREEP_MS;
            float rampFraction = constrain((float)spinElapsed / SPIN_RAMP_MS, 0.0f, 1.0f);
            int fullSpeed = baseSpeed + 10;
            int sp = (int)(SPIN_START_SPEED + rampFraction * (fullSpeed - SPIN_START_SPEED));

            if (lastValidSide <= 0) setMotorSpeed(-sp,  sp);
            else                    setMotorSpeed( sp, -sp);
        }
        return;
    }

    if (lineLost) {
        lineLost = false;
        resetPID();
        Serial.println("Line reacquired");
    }

    updateMotorControl(position);
}

void handleLookahead()
{
    if (millis() - stateStartTime < LOOKAHEAD_DRIVE_MS) {
        return;
    }

    readSensors();

    if (centerSeesLine()) {
        Serial.println("Lookahead: fake turn, continue straight");
        lookaheadActive = false;
        currentState    = STATE_FOLLOWING;
        resetPID();
    } else {
        Serial.println("Lookahead: real turn, spin recovery");
        lookaheadActive  = false;
        lineLost         = true;
        lineLostTime     = millis();
        currentState     = STATE_FOLLOWING;
    }
}

void handleIntersection()
{
    setMotorSpeed(baseSpeed, baseSpeed);
    if (millis() - stateStartTime > 120) {
        currentState = STATE_FOLLOWING; allBlackStartTime = 0; resetPID();
    }
}

void handleStopping()
{
    setMotorSpeed(0, 0);
    digitalWrite(2, LOW);
    if (millis() - stateStartTime > 5000) {
        currentState = STATE_IDLE; allBlackStartTime = 0;
        Serial.println("Ready for next run");
    }
}

void handleFakeBranch()   { currentState = STATE_FOLLOWING; }
void handleTightTurn()    { currentState = STATE_FOLLOWING; }
void handleStepRecovery() { currentState = STATE_FOLLOWING; }
void handleSpiralMode()   { currentState = STATE_FOLLOWING; }

// ========== MAIN LOOP ==========
void loop()
{
    ota_loop(server);
    bleLoop();

    switch (currentState) {
        case STATE_IDLE:          handleIdle();          break;
        case STATE_CALIBRATING:   handleCalibrating();   break;
        case STATE_STARTING:      handleStarting();      break;
        case STATE_FOLLOWING:     handleFollowing();     break;
        case STATE_LOOKAHEAD:     handleLookahead();     break;
        case STATE_FAKE_BRANCH:   handleFakeBranch();    break;
        case STATE_TURN_MODE:     handleTightTurn();     break;
        case STATE_STEP_RECOVERY: handleStepRecovery();  break;
        case STATE_SPIRAL_MODE:   handleSpiralMode();    break;
        case STATE_INTERSECTION:  handleIntersection();  break;
        case STATE_STOPPING:      handleStopping();      break;
    }

    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 3000) {
        Serial.print("State:"); Serial.print(currentState);
        Serial.print(" Base:"); Serial.print(baseSpeed);
        Serial.print(" Max:");  Serial.print(maxSpeed);
        Serial.print(" Cal:");  Serial.println(isCalibrated);
        lastDbg = millis();
    }
}

// ========== BLE CALLBACKS ==========
void onCalibrateRequest() { calibrationRequested = true; Serial.println("Cal requested"); }

void onStartRequest()
{
    Serial.print("START. Cal:"); Serial.println(isCalibrated ? "YES" : "NO");
    if (isCalibrated && currentState == STATE_IDLE) {
        currentState = STATE_STARTING; allBlackStartTime = 0;
        Serial.println("STARTING - place on ALL BLACK square");
    } else if (!isCalibrated) {
        Serial.println("Calibrate first!");
    } else {
        Serial.print("Bad state: "); Serial.println(currentState);
    }
}

void onStopRequest() { emergencyStop(); }

void onSpeedUpdate(int newBaseSpeed, int newMaxSpeed)
{
    baseSpeed = constrain(newBaseSpeed, 20, 100);
    maxSpeed  = constrain(newMaxSpeed,  40, 150);
    Serial.printf("Speed: Base=%d Max=%d\n", baseSpeed, maxSpeed);
}
