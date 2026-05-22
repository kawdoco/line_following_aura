#ifndef BLE_PID_H
#define BLE_PID_H

extern float kp;
extern float ki;
extern float kd;
extern bool robotStarted;
extern int baseSpeed;
extern int maxSpeed;

void onCalibrateRequest();
void onStartRequest();
void onStopRequest();
void onSpeedUpdate(int newBaseSpeed, int newMaxSpeed);

void bleSetup();
void bleLoop();

#endif