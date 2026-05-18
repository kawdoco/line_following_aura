#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <ESP2SOTA.h>
#include "ota.h"
#include "secrets.h"

void ota_begin(WebServer &server)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(1000);

    IPAddress IP(10, 10, 10, 1);
    IPAddress NMask(255, 255, 255, 0);
    WiFi.softAPConfig(IP, IP, NMask);

    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    ESP2SOTA.begin(&server);
    server.begin();

    Serial.println("OTA Ready");
}

void ota_loop(WebServer &server)
{
    server.handleClient();
}