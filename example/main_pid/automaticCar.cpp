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


bool running, started, stopped;
unsigned int bt_status, sensor_status;
VideoWriter color_videoWriter;
char key;
Mat binSignImg;

int main(int argc, char *argv[])
{
    printf("\n");
    // Init hardware
    GPIO_init();
    OpenNI_init();
    PCA9685_init();

	if( argc == 2)
		set_throttle_val = atoi(argv[1]);

    running = false, started = false, stopped = false;
    bt_status = sensor_status = 0;
    theta = 0;

    // Log
    color_videoWriter.open(color_filename, CV_FOURCC('M', 'J', 'P', 'G'), 8, Size(FRAME_WIDTH, FRAME_HEIGHT), true);

    // Calculate FPS
    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();
    
    // Run loop
    while (true)
    {
        printf("---------------------------\n");
        st = getTickCount();
        
        updateButtonStatus();
        updateSensorStatus();
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
        
        if (running)
        {
            // Update throttle val
            throttle_val = set_throttle_val;
            if (!started)
            {
                fprintf(stderr, "ON\n");
                started = true;
                stopped = false;
                throttle_val = set_throttle_val;
                api_set_FORWARD_control(pca9685, throttle_val);
            }
            
            // Update stream
            int readyStream = -1;
            rc = OpenNI::waitForAnyStream(streams, 1, &readyStream, -1);
            if (rc != STATUS_OK)
            {
                printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
                break;
            }

            // Load image
            colorStream.readFrame(&frame_color);
            analyzeFrame(frame_color, colorImg);
            
            // Preprocessing
            flip(colorImg, colorImg, 1);
            
            hist_equalize(colorImg);
            //medianBlur(colorImg, colorImg, KERNEL_SIZE);
            
            cvtColor(colorImg, hsvImg, CV_BGR2HSV);
            //cvtColor(colorImg, grayImg, CV_BGR2GRAY);
            
            get_mask(hsvImg, binImg, false, false, true); // black
	        bitwise_not(binImg, binImg);

            get_mask(hsvImg, binSignImg, true, true, false); // blue + red
            medianBlur(binSignImg, binSignImg, KERNEL_SIZE);
            imshow("binSignImg", binSignImg);

            cvtColor(colorImg, grayImg, CV_BGR2GRAY);

            // Process lane to get theta
            laneProcessing();
            
            signProcessing();

	        printf("theta: %d\n", int(theta));            

            imshow("colorImg", colorImg);
	        imshow("binImg", binImg);
            

            // Oh yeah... go go go :D
            api_set_FORWARD_control(pca9685, throttle_val);
            api_set_STEERING_control(pca9685, theta);

            // Log video
            if (!colorImg.empty())
                color_videoWriter.write(colorImg);
            
            
            et = getTickCount();
            fps = 1.0 / ((et - st) / freq);
            printf("FPS: %lf\n", fps);

            waitKey(10);
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
    // Release
    color_videoWriter.release();

    return 0;
}
