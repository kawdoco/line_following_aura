#include <WebServer.h>
#include "ota.h"

WebServer server(80);

void setup()
{
    Serial.begin(115200);
    ota_begin(server);
    pinMode(2,OUTPUT);
}

void loop()
{
    ota_loop(server);

    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
}