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

bool allowStopSign=true,hasBlueSign=false,hasRedSign=false;
bool running, started, stopped;
unsigned int bt_status, sensor_status;
VideoWriter color_videoWriter;
char key;
Point preCenterPoint;
Mat grayImg;
Point preLeft;
Point preRight;

int main(int argc, char *argv[])
{
    if (argc == 2)
        isDebug = true;
    else
        isDebug = false;

    printf("\n");
    // Init hardware
    GPIO_init();
    OpenNI_init();
    PCA9685_init();
    LCD_init();
    //init capture Stop Sign
    allowStopSign = true;
    
    running = false, started = false, stopped = false;
    bt_status = sensor_status = 0;
    theta = 0;
    turning =false;
    // Log
    color_videoWriter.open(color_filename, CV_FOURCC('M', 'J', 'P', 'G'), 8, Size(FRAME_WIDTH, FRAME_HEIGHT), true);

    // Calculate FPS
    double st = 0, et = 0;
    fps = 0;
    double freq = getTickFrequency();
    preCenterPoint = Point(FRAME_WIDTH / 2, (1 - CENTER_POINT_Y) * FRAME_HEIGHT * RATIO_HEIGHT_LANE_CROP);
    
//    hasSign = false;
    set_throttle_val = INIT_THROTTLE;
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
            api_set_STEERING_control(pca9685, theta);
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
            //hist_equalize(colorImg);
            //medianBlur(colorImg, colorImg, KERNEL_SIZE);
            cvtColor(colorImg, hsvImg, CV_BGR2HSV);
            
            cvtColor(colorImg, grayImg, CV_BGR2GRAY);

            // get lane binary image
            get_mask(hsvImg, binImg, false, false, true); // black
		    bitwise_not(binImg, binImg);

            // Get sign binary image
            get_mask(hsvImg, binBlueImg, true, false, false); // blue
            get_mask(hsvImg, binRedImg, false, true, false); // red
            // Mat element = getStructuringElement( MORPH_RECT,Size( 2*7+1, 2*7+1 ),Point( 7, 7 ) );
            // dilate(binRedImg,binRedImg,element);
            // erode( binRedImg,binRedImg, element );	
            
            medianBlur(binBlueImg, binBlueImg, KERNEL_SIZE);		
            medianBlur(binRedImg, binRedImg, KERNEL_SIZE);		
	    
            if (isDebug)
            {
                imshow("binBlueImg", binBlueImg);
                imshow("binRedImg", binRedImg);
            }
            
            // Process lane to get theta
            laneProcessing();
            
            // Process traffic sign
            signProcessing();

            printf("theta: %d\n", int(theta));            		

            // Oh yeah... go go go :D
            api_set_FORWARD_control(pca9685, throttle_val);
            api_set_STEERING_control(pca9685, theta);
            // Log video
            // if (!orgImg.empty())
            //     org_videoWriter.write(orgImg);
            if (isDebug && !colorImg.empty())
                color_videoWriter.write(colorImg);
            
            et = getTickCount();
            fps = int(1.0 / ((et - st) / freq));
            printf("FPS: %d\n", fps);
	        
            if (isDebug)
            {
                putText(colorImg, "FPS " + to_string(fps), Point(200, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
                imshow("bin", binImg);
                imshow("color", colorImg);
            }

            updateLCD();
            waitKey(1);
        }
        else
        {
            setupThrottle();
            updateLCD();
            theta = 0;
            throttle_val = 0;
            if (!stopped)
            {
                fprintf(stderr, "OFF\n");
                stopped = true;
                started = false;
            }
            api_set_FORWARD_control(pca9685, throttle_val);
            //usleep(200000);
        }
    }
    // Release
    //org_videoWriter.release();
    color_videoWriter.release();

    return 0;
}
