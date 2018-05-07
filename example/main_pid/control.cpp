#include "control.h"

Status rc;
Device device;
VideoStream colorStream;
VideoStream *streams[1];
VideoFrameRef frame_color;

string color_filename;
GPIO *gpio;
PCA9685 *pca9685;
I2C *i2c_device;
LCDI2C *lcd;

int sw1_stat, sw2_stat, sw3_stat, sw4_stat;
int sensor;
int set_throttle_val, throttle_val;

Sign blueSign, redSign;
bool hasBlueSign, hasRedSign;
int backupThrottle;
int fps;
bool isDebug;
//bool isCaptureStopSign;



void GPIO_init()
{
    printf("GPIO init...\n");
    // Switch input
    sw1_stat = 1;
    sw2_stat = 1;
    sw3_stat = 1;
    sw4_stat = 1;
    sensor = 1;

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

    streams[0] = {&colorStream};

    rc = OpenNI::initialize();

    color_filename = "color.avi";

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
        rc = colorStream.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK)
        {
            VideoMode color_mode = colorStream.getVideoMode();
            color_mode.setFps(30);
            color_mode.setResolution(FRAME_WIDTH, FRAME_HEIGHT);
            color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            colorStream.setVideoMode(color_mode);

            rc = colorStream.start();
            if (rc != STATUS_OK)
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
        }
        else
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
    }
}

void PCA9685_init()
{
    printf("PCA9685 init...\n");
    set_throttle_val = throttle_val = 0;
    pca9685 = new PCA9685;
    api_pwm_pca9685_init(pca9685);
    if (pca9685->error >= 0)
        api_set_FORWARD_control(pca9685, throttle_val);
    //backupThrottle = THROTTLE_VAL2;
}

void LCD_init()
{
    i2c_device = new I2C();
    lcd = new LCDI2C();
    i2c_device->m_i2c_bus = 2;

    lcd->LCDInit(i2c_device, 0x38, 20, 4);
    lcd->LCDBacklightOn();
    lcd->LCDCursorOn();

    lcd->LCDSetCursor(4, 0);
    lcd->LCDPrintStr("SOPHIA TEAM");
    
    lcd->LCDSetCursor(1, 1);
    lcd->LCDPrintStr("Sign: ");

    lcd->LCDSetCursor(1, 2);
    lcd->LCDPrintStr("Throttle:");

    lcd->LCDSetCursor(1, 3);
    lcd->LCDPrintStr("Sensor:");

    lcd->LCDSetCursor(12, 3);
    lcd->LCDPrintStr("FPS: ");
}

void updateButtonStatus()
{
    // Update status of physical buttons
    gpio->gpioGetValue(SW1_PIN, &bt_status);
    if (!bt_status)
    {
        if (bt_status != sw1_stat)
        {
            running = !running;
            sw1_stat = bt_status;
            throttle_val = set_throttle_val;
            backupThrottle = throttle_val;
        }
    }
    else
        sw1_stat = bt_status;

    gpio->gpioGetValue(SW4_PIN, &bt_status);
    if (!bt_status)
    {
        if (bt_status != sw4_stat)
        {
            running = !running;
            sw4_stat = bt_status;
            throttle_val = set_throttle_val;
            backupThrottle = throttle_val;
        }
    }
    else
        sw4_stat = bt_status;
}

void setupThrottle()
{

    gpio->gpioGetValue(SW2_PIN, &bt_status);
    if (!bt_status)
    {
        if (bt_status != sw2_stat)
        {
            sw2_stat = bt_status;
            set_throttle_val += 5;
        }
    }
    else
        sw2_stat = bt_status;

    gpio->gpioGetValue(SW3_PIN, &bt_status);
    if (!bt_status)
    {
        if (bt_status != sw3_stat)
        {
            sw3_stat = bt_status;
            set_throttle_val -= 5;
        }
    }
    else
        sw3_stat = bt_status;
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
        backupThrottle = throttle_val;
    }
    sensor = sensor_status;
}

void updateLCD()
{
    string s;
    // Sign
    lcd->LCDSetCursor(7, 1);
    if (hasBlueSign == true || hasBlueSign == true)
        s = "SIGN ";
    else
        s = "TURN ";
    bool prioritySign = true;
    switch (redSign.getClassID()) {
        case 3:
            s += "STOP ";
            break;
        default:
            s+= "NO SIGN";
            prioritySign = false;
            break;
    }
    if (!prioritySign) {

        switch (blueSign.getClassID()) {
            case 1:
                s += "LEFT";
                break;
            case 2:
                s += "RIGHT";
                break; 
            default:
                s+= "NO SIGN";
                break;
        }
    }
    lcd->LCDPrintStr(s.c_str());

    // Throttle val
    lcd->LCDSetCursor(11, 2);
    s = to_string(set_throttle_val) + "  ";
    lcd->LCDPrintStr(s.c_str());
    
    // Sensor
    lcd->LCDSetCursor(10, 3);
    s = to_string(sensor_status);
    lcd->LCDPrintStr(s.c_str());

    lcd->LCDSetCursor(17, 3);
    s = to_string(fps) + " ";
    lcd->LCDPrintStr(s.c_str());
}
double getSignTheta(int signID) {
    switch (signID) {
        case SIGN_STOP:
            return 0;
        case SIGN_LEFT:
            return -60;
        default:
            return 60;
    }
}
void signProcessing()
{
    // int signID = mySign.getClassID();
    // if (isDebug)
    // {
    //     if (signID == SIGN_LEFT)
    //         putText(colorImg, "Sign left", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
    //     else if (signID == SIGN_RIGHT)
    //         putText(colorImg, "Sign right", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
    //     else if (signID == SIGN_STOP)
    //         putText(colorImg, "Sign stop", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
    // }

    bool redDetected =redSign.detect(false); // red
    bool blueDetected = blueSign.detect(true); // blue
    int signID = 0;
    if (redDetected)
    {
        signID = redSign.recognize();
        if (isCaptureStopSign && signID == SIGN_STOP) {
            if (!hasRedSign)
            {
                backupThrottle = set_throttle_val;
                set_throttle_val = set_throttle_val * RATE_DECELERATION;
                hasRedSign = true;
            }
            theta = getSignTheta(SIGN_STOP);

            Rect signROI =redSign.getROI();
            rectangle(colorImg, Point(signROI.x, signROI.y), Point(signROI.x + signROI.width, signROI.y + signROI.height), Scalar(0, 0, 255), 2);
            
            cout << "Sign area: " << signROI.height * signROI.width << endl;

            if (signID == 3 && signROI.height * signROI.width >= MIN_AREA_SIGN_STOP) 
            {
                controlTurn(signID);
                redSign.resetClassID();
            }
            isCaptureStopSign = false;
        }
    }
    else if (blueDetected)
    {
        signID = blueSign.recognize();
        if (signID == SIGN_LEFT || signID == SIGN_RIGHT)
        {
            if (!hasBlueSign)
            {
                backupThrottle = set_throttle_val;
                set_throttle_val = set_throttle_val * RATE_DECELERATION;
                hasBlueSign = true; 
            }
            theta = getSignTheta(signID);

            Rect signROI = blueSign.getROI();
            rectangle(colorImg, Point(signROI.x, signROI.y), Point(signROI.x + signROI.width, signROI.y + signROI.height), Scalar(0, 0, 255), 2);
            cout << "Sign area: " << signROI.height * signROI.width << endl;

            if (signID != 3 && signROI.height * signROI.width >= MIN_AREA_SIGN_TURN)
            {
                controlTurn(signID);
                blueSign.resetClassID();
            }
            isCaptureStopSign = true;
        }
    }
}

void controlTurn(int signID)
{
    //hasSign = false;
    hasRedSign = false;
    hasBlueSign = false;
    updateLCD();

    if (signID == SIGN_LEFT)
    {
        theta = ALPHA_TURN;
        api_set_STEERING_control(pca9685, theta);
        if (isDebug)
        {
            putText(colorImg, "Turn left", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
            imshow("color", colorImg);
        }     
        usleep(TURN_TIME);
    }
    else if (signID == SIGN_RIGHT)
    {
        theta = -ALPHA_TURN;
        api_set_STEERING_control(pca9685, theta);
        if (isDebug)
        {
            putText(colorImg, "Turn right", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
            imshow("color", colorImg);
        }
        usleep(TURN_TIME);
    }
    else
    {
        api_set_FORWARD_control(pca9685, -30);
        usleep(300 * 1000);
        api_set_FORWARD_control(pca9685, 0);
        if (isDebug)
        {
            putText(colorImg, "Stop", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
            imshow("color", colorImg);
        }
        sleep(STOP_TIME);
    }
    set_throttle_val = backupThrottle;
}
