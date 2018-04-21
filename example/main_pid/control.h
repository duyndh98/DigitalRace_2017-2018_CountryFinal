#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "header.h"

    void GPIO_init();
    void OpenNI_init();
    void PCA9685_init();
    void updateButtonStatus();
    void updateSensorStatus();
    void updateKeyBoardInput();
    void controlTurn(PCA9685 *&pca9685, int dir);
    double PID(double fps, int xCar, int xCenter, double &previous_error, double &intergral);
    
#endif 