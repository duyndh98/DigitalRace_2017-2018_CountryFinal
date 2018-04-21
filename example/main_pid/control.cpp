#include "control.h"

void GPIO_init()
{
    printf("GPIO init...\n");
    gpio = new GPIO();
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
    usleep(10000);
}

void OpenNI_init()
{
    printf("OpenNI init...\n");
    rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Failed\n%s\n", OpenNI::getExtendedError());
        return;
    }
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
        return;

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
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
        }
        else
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
    }
}

void pca9685_init()
{
    printf("PCA9685 init...\n");
    pca9685 = new PCA9685;
    api_pwm_pca9685_init(pca9685);
    if (pca9685->error >= 0)
        api_set_FORWARD_control(pca9685, throttle_val);
}

void updateButtonStatus()
{
    // Update status of physical buttons
    gpio->gpioGetValue(SW4_PIN, &bt_status);
    if (!bt_status)
    {
        if (bt_status != sw4_stat)
        {
            running = !running;
            sw4_stat = bt_status;
            throttle_val = THROTTLE_VAL1;
            set_throttle_val = THROTTLE_VAL1;
        }
    }
    else
        sw4_stat = bt_status;

    gpio->gpioGetValue(SW1_PIN, &bt_status);
    if (!bt_status)
    {
        if (bt_status != sw1_stat)
        {
            running = !running;
            sw1_stat = bt_status;
            throttle_val = THROTTLE_VAL2;
            set_throttle_val = THROTTLE_VAL2;
        }
    }
    else
        sw1_stat = bt_status;
}

void updateSensorStatus()
{
    // Update status of distance-sensor
    gpio->gpioGetValue(SENSOR, &sensor_status);
    cout << "sensor: " << sensor_status << endl;
    if (sensor == 0 && sensor_status == 1)
    {
        running = true;
        throttle_val = set_throttle_val;
    }
    sensor = sensor_status;    
}

void updateKeyBoardInput()
{
    // Check input from keyboard
    key = getkey();
    if (key == 's')
    {
        running = !running;
        theta = 0;
        throttle_val = set_throttle_val;
    }
    if (key == 'f')
    {
        fprintf(stderr, "End process.\n");
        theta = 0;
        throttle_val = 0;
        api_set_FORWARD_control(pca9685, throttle_val);
        break;
    }
}

void controlTurn(PCA9685 *&pca9685, int dir)
{
    if (dir == SIGN_LEFT)
    {
        double theta = ALPHA * 78;
        api_set_STEERING_control(pca9685, theta);
        cout << "Turn Left <<<<<<<<<<<<<<<<<<<<\n";
        sleep(1);
        cout << "Normal" << endl;
    }
    else if (dir == SIGN_RIGHT)
    {
        double theta = -ALPHA * 78;
        api_set_STEERING_control(pca9685, theta);
        cout << "Turn Right >>>>>>>>>>>>>>>>>>>>\n";
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
