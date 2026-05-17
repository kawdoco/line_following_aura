#include <WiFi.h>
#include "wifi_manager.h"
#include "secrets.h"

void wifi_connect()
{
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi Connected");
    Serial.println(WiFi.localIP());
}