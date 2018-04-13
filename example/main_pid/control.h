#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "header.h"

    void GPIO_init(GPIO *&gpio);
    bool OpenNI_init(Status &rc, Device &device, VideoStream &depth, VideoStream &color);
    void controlTurn(PCA9685 *&pca9685, int dir);
    double PID(double fps, int xCar, int xCenter, double &previous_error, double &intergral);

#endif 