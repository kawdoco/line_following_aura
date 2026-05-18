#pragma once

#include <WebServer.h>

void ota_begin(WebServer &server);
void ota_loop(WebServer &server);