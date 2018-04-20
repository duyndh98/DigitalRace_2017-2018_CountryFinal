/**
    This code runs our car automatically and log video, controller (optional)
    Line detection method: Canny
    Targer point: vanishing point
    Control: pca9685
    
    You should understand this code run to image how we can make this car works from image processing coder's perspective.
    Image processing methods used are very simple, your task is optimize it.
    Besure you set throttle val to 0 before end process. If not, you should stop the car by hand.
    In our experience, if you accidental end the processing and didn't stop the car, you may catch it and switch off the controller physically or run the code again (press up direction button then enter).
**/
#include "header.h"
#include "image_processing.h"
#include "lane_detection.h"
#include "sign.h"
#include "control.h"

int main(int argc, char *argv[])
{
    // Setup input
    int sw1_stat = 1;
    int sw2_stat = 1;
    int sw3_stat = 1;
    int sw4_stat = 1;
    int sensor = 0;
    
    GPIO *gpio = new GPIO();
    GPIO_init(gpio);
    
    usleep(10000);

    // Init OpenNI
    Status rc;
    Device device;
    VideoStream color;
    OpenNI_init(rc, device,color);
    
    VideoFrameRef frame_color;
    VideoStream *streams[] = {&color};
    
    // Init video writer and log files
    bool is_save_file = true;
    
    // VideoWriter depth_videoWriter;
    VideoWriter org_videoWriter;
    VideoWriter color_videoWriter;
    VideoWriter gray_videoWriter;

    string org_filename = "org.avi";
    string color_filename = "color.avi";
    
    Mat orgImg, colorImg, hsvImg, grayImg, binImg, signMask;

    if (is_save_file)
    {
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        Size output_size(FRAME_WIDTH, FRAME_HEIGHT);
        
        org_videoWriter.open(org_filename, codec, 8, output_size, true);
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
    }
    
    // Init direction and ESC speed  //
    int set_throttle_val = 0, throttle_val = 0;
    double theta = 0;
    
    //  Init PCA9685 driver
    PCA9685 *pca9685 = new PCA9685();
    api_pwm_pca9685_init(pca9685);
    if (pca9685->error >= 0)
        api_set_FORWARD_control(pca9685, throttle_val);

    if (argc == 2)
        set_throttle_val = atoi(argv[1]);

    fprintf(stderr, "Initial throttle: %d\n", set_throttle_val);
    
    // Car running status
    bool running = false, started = false, stopped = false;

    // Calculate FPS
    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();

    unsigned int bt_status = 0;
    unsigned int sensor_status = 0;
    char key;

    // Use for lane detection
    Point centerPoint(FRAME_WIDTH / 2, (1 - CENTER_POINT_Y) * FRAME_HEIGHT);
    Point centerLeft(0, (1 - CENTER_POINT_Y) * FRAME_HEIGHT);
    Point centerRight(0, (1 - CENTER_POINT_Y) * FRAME_HEIGHT);
    bool isLeft, isRight;
    
    // Run loop
    while (true)
    {
        st = getTickCount();
        
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

        // Update status of distance-sensor
        gpio->gpioGetValue(SENSOR, &sensor_status);
        cout << "sensor: " << sensor_status << endl;
        
        if (sensor == 0 && sensor_status == 1)
        {
            running = true;
            throttle_val = set_throttle_val;
        }
        sensor = sensor_status;
        
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

        // Process
        if (running)
        {
            cout<< "---------------------------\n";
            throttle_val = set_throttle_val;
            // Check PCA9685 driver
            if (pca9685->error < 0)
            {
                cout << endl
                     << "Error: PWM driver" << endl
                     << flush;
                break;
            }
            if (!started)
            {
                fprintf(stderr, "ON\n");
                started = true;
                stopped = false;
                throttle_val = set_throttle_val;
                api_set_FORWARD_control(pca9685, throttle_val);
            }
            
            int readyStream = -1;
            rc = OpenNI::waitForAnyStream(streams, 1, &readyStream, -1);
            if (rc != STATUS_OK)
            {
                printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
                break;
            }

            // Load image
            color.readFrame(&frame_color);
            
            // Preprocessing
            analyzeFrame(frame_color, colorImg);
            colorImg.copyTo(orgImg);
            
            flip(colorImg, colorImg, 1);
            hist_equalize(colorImg);

            cvtColor(colorImg, hsvImg, CV_BGR2HSV);
            cvtColor(colorImg, grayImg, CV_BGR2GRAY);
            
            get_mask(hsvImg, binImg, false, false, true); // black
		    get_mask(hsvImg, signMask, true, true, false); // blue + red
            bitwise_not(binImg, binImg);

          //  Process lane to get center pPoint
            LaneProcessing(colorImg, binImg, centerPoint, centerLeft, centerRight, isLeft, isRight, theta);
            
            et = getTickCount();
            fps = 1.0 / ((et - st) / freq);
            cerr << "FPS: " << fps << '\n';

            imshow("bin", binImg);
            imshow("color", colorImg);            
                
            api_set_STEERING_control(pca9685, theta);
            api_set_FORWARD_control(pca9685, throttle_val);

            if (is_save_file)
            {
                if (!orgImg.empty())
                    org_videoWriter.write(orgImg);
                if (!colorImg.empty())
                    color_videoWriter.write(colorImg);
            }
            waitKey(1);
            
        }
        else
        {
            theta = 0;
            throttle_val = 0;
            if (!stopped)
            {
                fprintf(stderr, "OFF\n");
                stopped = true;
                started = false;
            }
            api_set_FORWARD_control(pca9685, throttle_val);
            sleep(1);
        }
    }
    //////////  Release //////////////////////////////////////////////////////
    if (is_save_file)
    {
        org_videoWriter.release();
        color_videoWriter.release();
    }
    return 0;
}
