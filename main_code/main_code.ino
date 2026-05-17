#include "wifi_manager.h"
#include "ota.h"
#include "secrets.h"

void setup()
{
  Serial.begin(115200);

  wifi_connect();
  ota_begin();
}

void loop()
{
  ota_loop();
}