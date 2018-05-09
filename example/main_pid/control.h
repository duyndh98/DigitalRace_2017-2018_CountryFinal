#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "header.h"
#include "sign.h"
#include "lane_detection.h"

void GPIO_init();
void OpenNI_init();
void PCA9685_init();
void LCD_init();
void updateButtonStatus();
void updateSensorStatus();
void updateKeyBoardInput();
void updateLCD();
void signProcessing();
void setupThrottle();
void controlTurn(int signID, Rect signROI);
double PID(double fps, int xCar, int xCenter, double &previous_error, double &intergral);
bool keyboardControl();

#endif 