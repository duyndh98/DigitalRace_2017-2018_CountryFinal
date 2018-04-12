#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "header.h"

    void controlTurn(PCA9685 *&pca9685, int dir);
    double PID(double fps, int x_car, int x_center, double &previous_error, double &intergral);

#endif 