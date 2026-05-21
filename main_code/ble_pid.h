#ifndef BLE_PID_H
#define BLE_PID_H

extern float kp;
extern float ki;
extern float kd;
extern bool robotStarted;

void onCalibrateRequest();

void bleSetup();
void bleLoop();

#endif