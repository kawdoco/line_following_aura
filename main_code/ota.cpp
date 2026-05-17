#include <WiFi.h>
#include <ArduinoOTA.h>
#include "ota.h"
#include "secrets.h"

void ota_begin()
{
    ArduinoOTA.setHostname(DEVICE_NAME);

    ArduinoOTA.onStart([]()
                       { Serial.println("OTA Start"); });

    ArduinoOTA.onEnd([]()
                     { Serial.println("OTA End"); });

    ArduinoOTA.onError([](ota_error_t error)
                       { Serial.println("OTA Error"); });

    ArduinoOTA.begin();

    Serial.println("OTA Ready");
}

void ota_loop()
{
    ArduinoOTA.handle();
}