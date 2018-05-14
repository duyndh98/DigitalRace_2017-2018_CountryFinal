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
double targetTimer;
double counterComeBack;
double counterStart;
bool turning;
Sign blueSign, redSign;

int backupThrottle;
int fps;
bool isDebug;
char key;
double freq, st_timeout_has_blue_sign, st_timeout_has_red_sign;

bool keyboardControl()
{
    if (key == ' ') // enter -> start control-mode
    {
        int alpha = 0;
        int throttle = 0;
        cout << "Control modeeeeeeeeeeeeeeeeeeeeeeeeeeee\n";
        while (1)
        {
            key = getkey();
            if (key == ' ') // re-entrer -> exit control-mode
                break;
            switch (key)
            {
            case 'w':
                api_set_FORWARD_control(pca9685, set_throttle_val);
                break;
            case 's':
                api_set_FORWARD_control(pca9685, -set_throttle_val);
                break;
            case 'a':
                api_set_STEERING_control(pca9685, ALPHA_TURN);
                break;
            case 'd':
                api_set_STEERING_control(pca9685, -ALPHA_TURN);
                break;
            case ENTER_KEY:
                api_set_FORWARD_control(pca9685, 0);
                api_set_STEERING_control(pca9685, 0);
                break;
            }
            waitKey(1);
        }
        api_set_FORWARD_control(pca9685, 0);
        api_set_STEERING_control(pca9685, 0);
        return true;
    }
    return false;
}

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

void updateLCD()
{
    string s;
    // Sign
    lcd->LCDSetCursor(0, 1);
    /*if (hasBlueSign == true || hasRedSign == true)
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
            case 2:
                break;
                s += "LEFT";
                s += "RIGHT";
                break; 
            default:
                //s+= "NO SIGN";
                break;
        }
    }*/
    if (!hasBlueSign && !hasRedSign)
        s = "NO SIGN";
    else if (hasRedSign)
    {
        if (turning)
            s = "TURN STOP";
        else
            s = "SIGN STOP";
    }

    else if (hasBlueSign)
    {
        if (blueSign.getClassID() == 1)
        {
            if (turning)
                s = "TURN LEFT";
            else
                s = "SIGN LEFT";
        }
        else if (blueSign.getClassID() == 2)
        {
            if (turning)
                s = "TURN RIGHT";
            else
                s = "SIGN RIGHT";
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
    //fps
    lcd->LCDSetCursor(17, 3);
    s = to_string(fps) + " ";
    lcd->LCDPrintStr(s.c_str());
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

double getWaitTurnTheta(int signID, Rect signROI)
{
    // switch (signID) {
    //     case SIGN_STOP:
    //         return 0;
    //     case SIGN_LEFT:
    //         return -60;
    //     default:
    //         return 60;
    // }
    if (signID == 3) // sign stop detected
        return theta;
    else // turn sign
    {
        Point signCenter(signROI.x + signROI.width / 2, signROI.y + signROI.height / 2);
        return getTheta(carPosition, signCenter);
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
    //allowStopSign = true;
    bool preHasSign = hasRedSign || hasBlueSign;

    if (redSign.detect(false) && redSign.recognize()) // red
    {
        hasRedSign = true;
        st_timeout_has_red_sign = getTickCount();
    } else
    {
        double et = getTickCount();
        if ((et - st_timeout_has_red_sign) / freq > TIMEOUT_HAS_RED_SIGN){
            allowStopSign = true;
        }
    }

    if (blueSign.detect(true) && blueSign.recognize()) // blue
    {
        hasBlueSign = true;
        st_timeout_has_blue_sign = getTickCount();
        counterStart = getTickCount();
    }
    else
    {
        double et = getTickCount();
        if ((et - st_timeout_has_blue_sign) / freq > TIMEOUT_HAS_BLUE_SIGN){
            hasBlueSign = false;
            laneMode = MIDDLE;
            cout << "NOT FOUND BLUE SIGN" << endl;
        }
    }
    if (!allowStopSign && hasRedSign)
        hasRedSign = false;
    if (hasRedSign && allowStopSign == true)
        hasBlueSign = false;
    /*
    if (hasBlueSign == false && hasRedSign == false)
        return false;
*/
    // from no sign -> has sign
    if (hasRedSign || hasBlueSign)
    {
        preLeft = Point(0, (1 - CENTER_POINT_Y) * FRAME_HEIGHT * RATIO_HEIGHT_LANE_CROP);
        preRight = Point(FRAME_WIDTH, (1 - CENTER_POINT_Y) * FRAME_HEIGHT * RATIO_HEIGHT_LANE_CROP);
        preCenterPoint = Point(FRAME_WIDTH / 2, (1 - CENTER_POINT_Y) * FRAME_HEIGHT * RATIO_HEIGHT_LANE_CROP);
        if (!preHasSign)
        {
            backupThrottle = set_throttle_val;
            set_throttle_val = set_throttle_val * RATE_DECELERATION;
        }

        Rect signROI(0, 0, 0, 0);
        int signID = NO_SIGN;
        if (hasBlueSign)
        {
            signID = blueSign.getClassID();
            signROI = blueSign.getROI();
        }
        if (hasRedSign)
        {
            signID = redSign.getClassID();
            signROI = redSign.getROI();
        }
        circle(colorImg, Point(signROI.x + signROI.width / 2, signROI.y + signROI.height / 2), 1, Scalar(255, 255, 0), 3);
        if (hasBlueSign && blueSign.getClassID())
        {
            // theta = -getWaitTurnTheta(signID, signROI) * ALPHA;
            // cout << "theta in control: " << theta << endl;
            laneMode = (blueSign.getClassID() == SIGN_LEFT) ? LEFT_FOLLOW : RIGHT_FOLLOW;
            rectangle(colorImg, Point(signROI.x, signROI.y), Point(signROI.x + signROI.width, signROI.y + signROI.height), Scalar(0, 0, 255), 2);
            cout << "sign area: " << signROI.height * signROI.width << endl;
        }

        if (signROI.height >= MIN_HEIGHT_SIGN_TURN)
        {
            turning = true;
            controlTurn(signID, signROI);
            turning = false;
        }
    }
    else
    {
        laneMode = MIDDLE;
    }
}

void controlTurn(int signID, Rect signROI)
{
    updateLCD();

    // if (signID == SIGN_LEFT)
    // {
    //     api_set_STEERING_control(pca9685, theta);
    //     if (isDebug)
    //     {
    //         putText(colorImg, "Turn left", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
    //         imshow("color", colorImg);
    //     }
    //     // theta = ALPHA_TURN;
    //     // usleep(TURN_TIME * 1000000);
    //     hasBlueSign = false;
    //     blueSign.resetClassID();
    //     allowStopSign = true;
    // }
    // else if (signID == SIGN_RIGHT)
    // {
    //     api_set_STEERING_control(pca9685, theta);
    //     if (isDebug)
    //     {
    //         putText(colorImg, "Turn right", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
    //         imshow("color", colorImg);
    //     }
    //     // theta = -ALPHA_TURN;
    //     // usleep(TURN_TIME * 1000000);
    //     hasBlueSign = false;
    //     blueSign.resetClassID();
    //     allowStopSign = true;
    // }
    if (signID == SIGN_STOP)
    {
        api_set_FORWARD_control(pca9685, -throttle_val);
        usleep(1000 * 400);
        api_set_FORWARD_control(pca9685, 0);
        if (isDebug)
        {
            putText(colorImg, "Stop", Point(0, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, CV_AA);
            imshow("color", colorImg);
        }
        sleep(STOP_TIME);
        hasRedSign = false;
        redSign.resetClassID();
        allowStopSign = false;
    }
    // if (signID == SIGN_RIGHT || signID == SIGN_LEFT)
    // {
    //     //targetTimer = TIME_RUN_CIRCLE;
    //     counterStart = getTickCount() / getTickFrequency();
    // }
    set_throttle_val = backupThrottle;
}
