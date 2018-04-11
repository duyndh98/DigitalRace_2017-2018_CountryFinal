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


void SetupInput(GPIO *gpio)
{
    gpio->gpioExport(SW1_PIN);
    gpio->gpioExport(SW2_PIN);
    gpio->gpioExport(SW3_PIN);
    gpio->gpioExport(SW4_PIN);
    gpio->gpioExport(SENSOR);

    gpio->gpioSetDirection(SW1_PIN, INPUT);
    gpio->gpioSetDirection(SW2_PIN, INPUT);
    gpio->gpioSetDirection(SW3_PIN, INPUT);
    gpio->gpioSetDirection(SW4_PIN, INPUT);
    gpio->gpioSetDirection(SENSOR, INPUT);
    
}
