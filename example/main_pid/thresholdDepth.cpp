
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480


Mat grayImg;
int upper__bound= 55, lower__bound=0;
int thresh_area_min= 20 ,thresh_area_max = 70000;
void on_upper__bound_thresh_trackbar(int, void *)
{
    upper__bound = max(upper__bound, lower__bound + 1);
    setTrackbarPos("upper__bound", "Threshold Selection", upper__bound);
}

void on_lower__bound_thresh_trackbar(int, void *)
{
    lower__bound = min(upper__bound - 1, lower__bound);
    setTrackbarPos("lower__bound", "Threshold Selection", lower__bound);
}

void on_thresh_area_min_thresh_trackbar(int, void *)
{
    thresh_area_min = min(thresh_area_max - 1, thresh_area_min);
    setTrackbarPos("thresh_area_min", "Threshold Selection", thresh_area_min);
}

void on_thresh_area_max_thresh_trackbar(int, void *)
{
    thresh_area_max = max(thresh_area_max, thresh_area_min + 1);
    setTrackbarPos("thresh_area_max", "Threshold Selection", thresh_area_max);
}

void
mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes,
                      cv::Mat &image,
                      std::vector<cv::Rect> &outputBoxes)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    cv::Size scaleFactor(15,15);

    for (int i = 0; i < inputBoxes.size(); i++)
    {
        cv::Rect box = inputBoxes.at(i) + scaleFactor;
        cv::rectangle(mask, box, cv::Scalar(255), CV_FILLED); // Draw filled bounding boxes on mask
    }

    std::vector< std::vector< cv::Point> > contours;
    // Find contours in mask
    // If bounding boxes overlap, they will be joined by this function call
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int j = 0; j < contours.size(); j++)
    {
        outputBoxes.push_back(cv::boundingRect(contours.at(j)));
    }
}

void
truncate(Mat &src, Mat &dst,
         int lower_bound, int upper_bound)
{
    for(int y = 0; y < src.rows; y++)
    {
        for(int x = 0; x < src.cols; x++)
        {
            uchar &d = src.at<uchar>(y,x);
            uchar &e = dst.at<uchar>(y,x);

            if( d > lower_bound && d < upper_bound )
                e = 255;
            else
                e = 0;
        }
    }
}

bool
api_kinect_cv_get_obtacle_rect( Mat& depthMap,
                                vector< Rect > &output_boxes,
                                Rect &roi,
                                int lower__bound,
                                int upper__bound, int thresh_area_min, int thresh_area_max)
{

    //int thresh_area_min = 100;//300
    //int thresh_area_max = 400;//400

    int erosion_type = MORPH_ELLIPSE;
    int erosion_size = 1;


    Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    Mat tmpDepthMap = depthMap(roi).clone();
    Mat binImg = depthMap(roi).clone();

    truncate( tmpDepthMap, binImg, lower__bound, upper__bound);

    erode ( binImg, binImg, element );
    dilate( binImg, binImg, element );

//    imshow( "depthMap", tmpDepthMap );
//    imshow( "depthMapBin", binImg );

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( binImg, contours, hierarchy, CV_RETR_EXTERNAL,
                  CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect1( contours.size() );
    vector<Rect> boundRect2;
    //vector<vector<Point> > contourMax =contours[0];

    for( int i = 0; i < contours.size(); i++ )
    {
    //if (contourMax.area() < countours[i]) contourMax = contours[i];
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect1[i] = boundingRect( Mat(contours_poly[i]) );
    }

    Mat binImg1 = Mat::zeros(depthMap.size(), CV_8UC1);
    Mat binImg2 = Mat::zeros(depthMap.size(), CV_8UC1);
    



    Rect boundRectMax = Rect(0,0,0,0);

    for( int i = 0; i< contours.size(); i++ )
    {
        if( (boundRect1[i].area() > thresh_area_min)&&(boundRect1[i].area()<thresh_area_max))
            boundRect2.push_back(boundRect1[i]);
    }

    for( int i = 0; i< boundRect2.size(); i++ )
    {
        rectangle( binImg1, boundRect2[i], Scalar( 255), CV_FILLED);
    }

//    imshow( "depthMapBin1", binImg1 );

    vector<Rect> tmp_outputBoxes;

    mergeOverlappingBoxes( boundRect2, binImg2, tmp_outputBoxes);

    for( int i = 0; i< tmp_outputBoxes.size(); i++ )
    {
        if( tmp_outputBoxes[i].area() > 1.5*thresh_area_min )
            output_boxes.push_back( tmp_outputBoxes[i] + roi.tl());
    }
    
    if (contours.size() == 0)
	return false;

return true;

}

void
api_kinect_cv_center_rect_gen(
        vector< Rect > &rects,
        int frame_width,
        int frame_height
        )
{
    int start_y = 140;
    int end_y   = 320;

    int x_max = 200;
    int x_min = 160;


    for( int i = 0; i < 16; i++ )
    {
        int x, y, w, h;

        x = frame_width / 2 - x_max/2 + i*5;

        w = x_max - i*10;

        y = frame_height - start_y - i*10;

        h = 10;

        Rect roi( x, y, w, h);
        rects.push_back( roi );
    }
}

bool thoidiemre(Mat &depthMap)
{
	int slice_nb = 3;
    int lower_slice_idx = 3;
    int upper_slice_idx = lower_slice_idx + slice_nb;
    //int lower__bound = DIST_MIN + lower_slice_idx * SLICE_DEPTH;
    //int upper__bound = lower__bound + slice_nb*SLICE_DEPTH;
    //Mat grayImage = colorImg.clone();
    vector< Rect > rects;
    Rect intersect;
    vector< Rect > output_boxes;

    Rect roi_1 = Rect(0, VIDEO_FRAME_HEIGHT/4,
                    VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT/2);
    Rect roi_2(0, VIDEO_FRAME_HEIGHT*1/3, VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT*1/3);

    int frame_width = VIDEO_FRAME_WIDTH;
    int frame_height = VIDEO_FRAME_HEIGHT;

    //api_kinect_cv_get_images(capture, depthMap , grayImage); //get depth
    //namedWindow("DepthImg", 1);


//    Mat crop_grayImage = grayImage(roi_1);
    Mat crop_depthMap = depthMap(roi_1);

    api_kinect_cv_get_obtacle_rect( depthMap, output_boxes, roi_2, lower__bound, upper__bound, thresh_area_min, thresh_area_max); 

    //chinh lai trong api.cpp va file .h, truyen tham chieu cho 4 tham so cuoi
    Mat binImg = Mat::zeros(depthMap.size(), CV_8UC1);

    api_kinect_cv_center_rect_gen( rects, frame_width, frame_height);
    Rect center_rect = rects[lower_slice_idx];
    //center_rect = center_rect + Size(0, slice_nb*SLICE_DEPTH); // lay center_rect
    if (output_boxes.size()==0) {
        cout <<"NONE"<<endl;
        return false;
    }
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



        /*//lay toa do, rong, cao.
    xL = intersect.x;
    xR = intersect.x + intersect.width;
    y = intersect.y;
    w = intersect.width;
    h = intersect.height;*/

    imshow( "BoundingRect", binImg );
//    if(!grayImage.empty())
//        imshow( "gray", crop_grayImage );

    if (!depthMap.empty() )
        imshow( "depth", crop_depthMap );
    cout <<"YEP"<<endl;

    return true;
}

int
api_kinect_cv_get_images(VideoCapture &capture,
        Mat &depthMap,
        Mat &grayImage)
{
    if( !capture.isOpened() )
    {
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    if( !capture.grab() )
    {
        cout << "Can not grab images." << endl;
        return -1;
    }
    else
    {
        Mat depth;
        if( capture.retrieve( depth, CV_CAP_OPENNI_DEPTH_MAP ) )
        {
            //depthMap = depth.clone();
            const float scaleFactor = 0.05f;
            depth.convertTo( depthMap, CV_8UC1, scaleFactor );
        }
        else
        {
            cout<< endl<< "Error: Cannot get depthMap image";
            return -1;
        }

/*        if( !capture.retrieve( grayImage, CV_CAP_OPENNI_GRAY_IMAGE ) )
        {
            cout<< endl<< "Error: Cannot get gray image";
            return -1;
        }
*/    }

    return 0;
}

int main()
{



    Mat depthMap,grayImage,bgrImage;
	VideoCapture capture;
	capture.open( CV_CAP_OPENNI2 );
	if( !capture.isOpened() )
    {
        cout << "Can not open a capture object." << endl;
        return -1;
    }
	capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
    capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 0 );
	

	//call funtion get depthImg

    namedWindow("Threshold Selection", WINDOW_NORMAL);
    createTrackbar("lower__bound", "Threshold Selection", &lower__bound, 300, on_lower__bound_thresh_trackbar);
    createTrackbar("upper__bound", "Threshold Selection", &upper__bound, 300, on_upper__bound_thresh_trackbar);
    createTrackbar("thresh_area_min", "Threshold Selection", &thresh_area_min, 400, on_thresh_area_min_thresh_trackbar);
    //createTrackbar("thresh_area_max", "Threshold Selection", &thresh_area_max, 800, on_thresh_area_max_thresh_trackbar);
     double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();
/*	for(;;)
{
    Mat depthMap1;
    Mat bgrImage1;
    capture.grab();
    capture.retrieve( depthMap1, CAP_OPENNI_DEPTH_MAP );
    //capture.retrieve( bgrImage1, CAP_OPENNI_BGR_IMAGE );
	//imshow("capture Img",bgrImage1);
	imshow("depth: ",depthMap1);
	cout << "FPS    " << capture.get( CAP_OPENNI_IMAGE_GENERATOR+CAP_PROP_FPS ) << endl;

    waitKey( 1 );
}
*/
	while ((char)waitKey(1) != 'q')
    {
		st = getTickCount();
	api_kinect_cv_get_images( capture, depthMap, grayImage);
	//imshow("capture Img",grayImage);
	//cout<<"width ="<<grayImage.cols<<"-height-="<<grayImage.rows<<endl;
	
//	if( !capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE ) )
//	        {
//	            cout<< endl<< "Error: Cannot bgr gray image";
//	            return -1;
//	        }
        thoidiemre(depthMap);
	et = getTickCount();
            fps = 1.0 / ((et - st) / freq);
            printf("FPS: %lf\n", fps);
    }

    return 0;
}
