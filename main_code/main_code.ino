#include <Arduino.h>
#include <WebServer.h>
#include "ble_pid.h"
#include "ota.h"

WebServer server(80);

void setup()
{
    Serial.begin(115200);
    ota_begin(server);
    bleSetup();
    pinMode(2,OUTPUT);
}

void loop()
{
    ota_loop(server);

 bleLoop();
Serial.print("KP: ");
    Serial.print(kp);

    Serial.print(" | KI: ");
    Serial.print(ki);

    Serial.print(" | KD: ");
    Serial.print(kd);

    Serial.print(" | ROBOT: ");

    if (robotStarted)
    {
        Serial.println("STARTED");
    }
    else
    {
        Serial.println("STOPPED");
    }

    delay(1000);

}