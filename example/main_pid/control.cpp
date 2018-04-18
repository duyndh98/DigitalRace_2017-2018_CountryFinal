#include "control.h"

void GPIO_init(GPIO *&gpio)
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

bool OpenNI_init(Status &rc, Device &device, VideoStream &color)
{
    rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 0;
    }
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
        return 0;

    // color video stream init
    if (device.getSensorInfo(SENSOR_COLOR) != NULL)
    {
        rc = color.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK)
        {
            VideoMode color_mode = color.getVideoMode();
            color_mode.setFps(30);
            color_mode.setResolution(FRAME_WIDTH, FRAME_HEIGHT);
            color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            color.setVideoMode(color_mode);

            rc = color.start();
            if (rc != STATUS_OK)
            {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else
        {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }

    return 1;
}

void controlTurn(PCA9685 *&pca9685, int dir)
{
    if (dir == SIGN_LEFT)
    {
        double theta = ALPHA * 78;
        api_set_STEERING_control(pca9685, theta);
        cout << "Turn Left=================================================" << endl;
        sleep(1);
        cout << "Normal" << endl;
    }
    else if (dir == SIGN_RIGHT)
    {
        double theta = -ALPHA * 78;
        api_set_STEERING_control(pca9685, theta);
        cout << "Turn Right=================================================" << endl;
        sleep(1);
        cout << "Normal" << endl;
    }
}

double PID(double fps, int xCar, int xCenter, double &previous_error, double &intergral)
{
    double dt = 1.0/fps;
    double error = xCenter - xCar;
    intergral = intergral + (dt * error);
    double derivative = (error - previous_error) / dt;
    double output = (KP * error) + (KI * intergral)+ (KD * derivative);
    previous_error = error;
    return output + xCar;
}

