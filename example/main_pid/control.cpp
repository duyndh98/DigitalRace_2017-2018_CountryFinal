#include "control.h"

void controlTurn(PCA9685 *&pca9685, int dir)
{
    if (dir == SIGN_LEFT)
    {
        double theta = ALPHA * 78;
        api_set_STEERING_control(pca9685, theta);
        cout << "Turn Left==================================================" << endl;
        sleep(1);
        cout << "Normal" << endl;
    }
    else if (dir == SIGN_RIGHT)
    {
        double theta = -ALPHA * 78;
        api_set_STEERING_control(pca9685, theta);
        cout << "Turn Right==================================================" << endl;
        sleep(1);
        cout << "Normal" << endl;
    }
}

double PID(double fps, int x_car, int x_center, double &previous_error, double &intergral)
{
    double dt = 1.0/fps;
    double error = x_center - x_car;
    intergral = intergral + (dt * error);
    double derivative = (error - previous_error) / dt;
    double output = (KP * error) + (KI * intergral)+ (KD * derivative);
    previous_error = error;
    return output + xCar;
}

