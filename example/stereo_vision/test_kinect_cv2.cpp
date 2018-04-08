
#include "api_kinect_cv.h"

#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480


int
getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int
main( int argc, char** argv )
{
	namedWindow("Control");
    VideoCapture capture;
	int iLowH = 70;
	int iHighH = 100;
	int iLowS = 200; 
	int iHighS = 255;
	int iLowV = 20;
	int iHighV = 130;
	int iLowArea = 10000;
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);
	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);
	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);	
	createTrackbar("LowArea", "Control", &iLowArea, 30000);
    capture.open( CV_CAP_OPENNI2 );
    //if( !capture.isOpened() )
    //capture.open( CV_CAP_OPENNI );

    if( !capture.isOpened() )
    {
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

    capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 0 );

    int codec = CV_FOURCC('D','I','V', 'X');

    int frame_width = capture.get( CV_CAP_PROP_FRAME_WIDTH );
    int frame_height = capture.get( CV_CAP_PROP_FRAME_HEIGHT );

    Size output_size(frame_width, frame_height);

    string gray_filename = "gray.avi";
    string depth_filename = "depth.avi";
	string bgr_filename = "bgr.avi";

    VideoWriter gray_videoWriter;
    VideoWriter depth_videoWriter;
    VideoWriter disp_videoWriter;
	VideoWriter bgr_videoWriter;

    Mat depthMap;
    Mat grayImage;
	Mat bgrImage;

    Mat crop_depthMap;
    Mat crop_grayImage;

    Rect roi = Rect(0, VIDEO_FRAME_HEIGHT/4,
                    VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT/2);

    char key = 0;

    bool start = false;
    bool stop = false;


    bool is_show = true;
    bool is_save_file = true;

    if(is_save_file)
    {
        gray_videoWriter.open(gray_filename, codec, 24, output_size, false);
        depth_videoWriter.open(depth_filename, codec, 24, output_size, false);
		bgr_videoWriter.open(bgr_filename, codec, 24, output_size, true);
    }

    vector< Rect > rects;
    api_kinect_cv_center_rect_gen( rects, frame_width, frame_height);

    int slice_nb = 3;
    int lower_slice_idx = 3;
    int upper_slice_idx = lower_slice_idx + slice_nb;
    //int lower_bound = DIST_MIN + lower_slice_idx * SLICE_DEPTH;
    //int upper_bound = lower_bound + slice_nb*SLICE_DEPTH;
	int lower_bound = 30;
	int upper_bound = 70;	

    Rect center_rect = rects[lower_slice_idx];
    center_rect = center_rect + Size(0, slice_nb*SLICE_DEPTH);

    Rect intersect;

    cout<< " Press s to start and f to stop"<< endl<< flush;

    while( true )
    {

        key = getkey();
        if( key == 's')
            start = true;
        if( key == 'f')
            stop = true;
        if( stop ) break;

        if( start )
        {
            api_kinect_cv_get_images( capture, depthMap, grayImage);
			if( !capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE ) )
	        {
	            cout<< endl<< "Error: Cannot bgr gray image";
	            return -1;
	        }
		Mat imgHSV;
            crop_grayImage = grayImage(roi);
            crop_depthMap = depthMap(roi);
		cvtColor(bgrImage, imgHSV, COLOR_BGR2HSV);
		
		Mat imgThresholded;
		
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		Moments oMoments = moments(imgThresholded);
		
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
		
		if(dArea > iLowArea) {
			cout << "Not obstacle: " << dArea << endl;
		}
		else {
			int posX = dM10/dArea;
			int posY = dM01/dArea;
					circle(bgrImage, Point(posX,posY), 30, Scalar(255,0,0), -1);
			cout << "Obstacle: " << posX << ", " << posY << endl;
		}


		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Obstacle", bgrImage);
		//imshow("Original", imgOriginal); //show the original image
            if( is_show )
            {
                Rect roi(0, 132, 640, 238);
                vector< Rect > output_boxes;
                api_kinect_cv_get_obtacle_rect( depthMap, output_boxes, roi,
                                                lower_bound, upper_bound );
                Mat binImg = Mat::zeros(depthMap.size(), CV_8UC1);

//                rectangle( binImg, center_rect, Scalar( 200));

                for( int i = 0; i< output_boxes.size(); i++ )
                {
//                    rectangle( binImg, output_boxes[i], Scalar( 255) );

                    //intersect = output_boxes[i] & center_rect;
			intersect = output_boxes[i];
                    
                    if( intersect.area() != 0 )
                    {
                        rectangle( binImg, intersect, Scalar( 255) );
                        //cout<< endl<< "Has Collision detect"<< flush;
                    }
                }

                imshow( "BoundingRect", binImg );

                if(!grayImage.empty())
                    imshow( "gray", crop_grayImage );

                if (!depthMap.empty() )
                    imshow( "depth", crop_depthMap );


                key = waitKey( 10 );

                if( key == 27 )
                    break;
            }

            if(is_save_file)
            {
                if(!grayImage.empty())
                    gray_videoWriter.write(grayImage);

                if (!depthMap.empty() )
                    depth_videoWriter.write(depthMap);

				if (!bgrImage.empty())
					bgr_videoWriter.write(bgrImage);
            }
        }
    }


    if(is_save_file )
    {
        gray_videoWriter.release();
        depth_videoWriter.release();
        disp_videoWriter.release();
		bgr_videoWriter.release();
    }

}
