#include "ble_pid.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID    "12345678-1234-1234-1234-123456789abc"
#define CHAR_P_UUID     "12345678-1234-1234-1234-123456789001"
#define CHAR_I_UUID     "12345678-1234-1234-1234-123456789002"
#define CHAR_D_UUID     "12345678-1234-1234-1234-123456789003"
#define CHAR_START_UUID "12345678-1234-1234-1234-123456789004"
#define CHAR_CALIBRATE_UUID "12345678-1234-1234-1234-123456789005"
#define CHAR_BASE_SPEED_UUID "12345678-1234-1234-1234-123456789006"
#define CHAR_MAX_SPEED_UUID  "12345678-1234-1234-1234-123456789007"

BLECharacteristic *charP, *charI, *charD, *charStart, *charCalibrate, *charBaseSpeed, *charMaxSpeed;

bool deviceConnected = false;

float kp = 0.35;
float ki = 0.08;
float kd = 0.12;
bool robotStarted = false;

int baseSpeed = 45;
int maxSpeed = 70;

float readFloat(BLECharacteristic *c)
{
    uint8_t *data = c->getData();
    float val;
    memcpy(&val, data, 4);
    return val;
}

int readInt(BLECharacteristic *c)
{
    uint8_t *data = c->getData();
    int val;
    memcpy(&val, data, 4);
    return val;
}

class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *s)
    {
        deviceConnected = true;
        Serial.println(">> CLIENT CONNECTED");
    }

    void onDisconnect(BLEServer *s)
    {
        deviceConnected = false;
        Serial.println(">> CLIENT DISCONNECTED");
        s->startAdvertising();
    }
};

class PIDCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c)
    {
        float val = readFloat(c);
        String uuid = c->getUUID().toString().c_str();

        if (uuid == CHAR_P_UUID)
        {
            kp = val;
            Serial.printf("P = %.2f\n", kp);
        }
        else if (uuid == CHAR_I_UUID)
        {
            ki = val;
            Serial.printf("I = %.3f\n", ki);
        }
        else if (uuid == CHAR_D_UUID)
        {
            kd = val;
            Serial.printf("D = %.2f\n", kd);
        }
        else if (uuid == CHAR_BASE_SPEED_UUID)
        {
            int speed = readInt(c);
            onSpeedUpdate(speed, maxSpeed);
            Serial.printf("Base Speed = %d\n", speed);
        }
        else if (uuid == CHAR_MAX_SPEED_UUID)
        {
            int speed = readInt(c);
            onSpeedUpdate(baseSpeed, speed);
            Serial.printf("Max Speed = %d\n", speed);
        }
    }
};

class StartCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c)
    {
        uint8_t cmd = c->getData()[0];
        if (cmd == 1)
        {
            robotStarted = true;
            onStartRequest();
            Serial.println(">> START ROBOT");
        }
        else
        {
            robotStarted = false;
            onStopRequest();
            Serial.println(">> STOP ROBOT");
        }
    }
};

class CalibrateCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c)
    {
        uint8_t cmd = c->getData()[0];
        if (cmd == 1)
        {
            onCalibrateRequest();
            Serial.println(">> CALIBRATE REQUESTED");
        }
    }
};

void bleSetup()
{
    Serial.println("Booting BLE...");
    BLEDevice::init("PID-Robot");
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    BLEService *service = server->createService(SERVICE_UUID);

    auto mkChar = [&](const char *uuid)
    {
        return service->createCharacteristic(uuid, BLECharacteristic::PROPERTY_WRITE);
    };

    charP = mkChar(CHAR_P_UUID);
    charI = mkChar(CHAR_I_UUID);
    charD = mkChar(CHAR_D_UUID);
    charStart = mkChar(CHAR_START_UUID);
    charCalibrate = mkChar(CHAR_CALIBRATE_UUID);
    charBaseSpeed = mkChar(CHAR_BASE_SPEED_UUID);
    charMaxSpeed = mkChar(CHAR_MAX_SPEED_UUID);

    charP->setCallbacks(new PIDCallback());
    charI->setCallbacks(new PIDCallback());
    charD->setCallbacks(new PIDCallback());
    charStart->setCallbacks(new StartCallback());
    charCalibrate->setCallbacks(new CalibrateCallback());
    charBaseSpeed->setCallbacks(new PIDCallback());
    charMaxSpeed->setCallbacks(new PIDCallback());

    service->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->start();
    Serial.println("Advertising as 'PID-Robot'");
}

void bleLoop() {}