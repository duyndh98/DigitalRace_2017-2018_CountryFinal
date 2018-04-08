
#include "api_kinect_cv.h"

void
api_kinect_cv_disparity2color( const Mat& gray,
                   Mat& rgb,
                   double maxDisp,
                   float S, float V )
{
    CV_Assert( !gray.empty() );
    CV_Assert( gray.type() == CV_8UC1 );

    if( maxDisp <= 0 )
    {
        maxDisp = 0;
        minMaxLoc( gray, 0, &maxDisp );
    }

    rgb.create( gray.size(), CV_8UC3 );
    rgb = Scalar::all(0);
    if( maxDisp < 1 )
        return;

    for( int y = 0; y < gray.rows; y++ )
    {
        for( int x = 0; x < gray.cols; x++ )
        {
            uchar d = gray.at<uchar>(y,x);
            unsigned int H = ((uchar)maxDisp - d) * 240 / (uchar)maxDisp;

            unsigned int hi = (H/60) % 6;
            float f = H/60.f - H/60;
            float p = V * (1 - S);
            float q = V * (1 - f * S);
            float t = V * (1 - (1 - f) * S);

            Point3f res;

            if( hi == 0 ) //R = V,  G = t,  B = p
                res = Point3f( p, t, V );
            if( hi == 1 ) // R = q, G = V,  B = p
                res = Point3f( p, V, q );
            if( hi == 2 ) // R = p, G = V,  B = t
                res = Point3f( t, V, p );
            if( hi == 3 ) // R = p, G = q,  B = V
                res = Point3f( V, q, p );
            if( hi == 4 ) // R = t, G = p,  B = V
                res = Point3f( V, p, t );
            if( hi == 5 ) // R = V, G = p,  B = q
                res = Point3f( q, p, V );

            uchar b = (uchar)(std::max(0.f, std::min (res.x, 1.f)) * 255.f);
            uchar g = (uchar)(std::max(0.f, std::min (res.y, 1.f)) * 255.f);
            uchar r = (uchar)(std::max(0.f, std::min (res.z, 1.f)) * 255.f);

            rgb.at<Point3_<uchar> >(y,x) = Point3_<uchar>(b, g, r);
        }
    }
}

float
api_kinect_cv_get_max_disparity( VideoCapture& capture )
{
    const int minDistance = 400; // mm
    float b = (float)capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_BASELINE ); // mm
    float F = (float)capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ); // pixels
    return b * F / minDistance;
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

        if( !capture.retrieve( grayImage, CV_CAP_OPENNI_GRAY_IMAGE ) )
        {
            cout<< endl<< "Error: Cannot get gray image";
            return -1;
        }
    }

    return 0;
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
                                int lower_bound,
                                int upper_bound)
{

    int thresh_area_min = 100;//300
	int thresh_area_max = 400;//400

    int erosion_type = MORPH_ELLIPSE;
    int erosion_size = 1;


    Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    Mat tmpDepthMap = depthMap(roi).clone();
    Mat binImg = depthMap(roi).clone();

    truncate( tmpDepthMap, binImg, lower_bound, upper_bound);

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
	
	Rect boundRectMax = boundRect1[0];
    for( int i = 0; i< contours.size(); i++ )
    {
        if( (boundRect1[i].area() > thresh_area_min))
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
if (contours.size() == 0 ) return false;
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
void get_obtacle(Mat depthMap, int &xL, int &xR, int &y, int &w, bool &test_obt)
{
    ////
   /// imshow("depthMap",depthMap);
////
    using namespace std;
    using namespace cv;
     // Init//
    int slice_nb = 3;
    int lower_slice_idx = 3;
    int upper_slice_idx = lower_slice_idx + slice_nb;
    //int lower_bound = DIST_MIN + lower_slice_idx * SLICE_DEPTH;
    //int upper_bound = lower_bound + slice_nb*SLICE_DEPTH;
    int lower_bound = 5;//20; /////xa gan
    int upper_bound = 50;//50;
	test_obt= false;

    
    //Mat depthMap;
    Mat grayImage;
    vector< Rect > rects;
    Rect intersect;
    vector< Rect > output_boxes;

    Rect roi_1 = Rect(0, grayImage.rows/4,
                    grayImage.cols, grayImage.rows/2);
    Rect roi_2(0,32 , 320, 118);

    int frame_width = 320; //capture.get( CV_CAP_PROP_FRAME_WIDTH );
    int frame_height = 240; //capture.get( CV_CAP_PROP_FRAME_HEIGHT );

    //api_kinect_cv_get_images(capture, depthMap , grayImage); //get depth

    Mat crop_grayImage = grayImage(roi_1);
    Mat crop_depthMap = depthMap(roi_1);

    test_obt = api_kinect_cv_get_obtacle_rect( depthMap, output_boxes, roi_2, lower_bound, upper_bound );/////////--kiem tra co vat can hay khong 
	if (test_obt == true) cout<< "co vat can" << endl;
	else cout<<"khong vat can\n";
//--------------------------------------------------------------
	
    Mat binImg = Mat::zeros(depthMap.size(), CV_8UC1);

    api_kinect_cv_center_rect_gen( rects, frame_width, frame_height);
    Rect center_rect = rects[lower_slice_idx];
    center_rect = center_rect + Size(0, slice_nb*SLICE_DEPTH); // lay center_rect
	int temp=0;

    for( int i = 0; i< output_boxes.size(); i++ )
        {
//                    rectangle( binImg, output_boxes[i], Scalar( 255) );
            //intersect = output_boxes[i] & center_rect;
            intersect = output_boxes[i];
            if( intersect.area() != 0 && ((float)intersect.width/intersect.height<1.2 || (float)intersect.width/intersect.height>0.2))
               {
                    rectangle( binImg, intersect, Scalar( 255) );
                    //cout<< endl<< "Has Collision detect"<< flush;
			temp++;
			cout <<temp;
                }

		else cout <<"intersect = 0"<<endl;
        }
        //lay toa do, rong, cao.
    xL = intersect.x;
    xR = intersect.x + intersect.width;
    y = intersect.y;
    w = intersect.width;
    //int h = intersect.height;
	
	

    imshow( "BoundingRect", binImg );
    if(!grayImage.empty())
        imshow( "gray", crop_grayImage );
/*   if (!depthMap.empty() )
        imshow( "depth", crop_depthMap );
*/
}
void PointCenter_Displacement(int &xTam, int xL, int xR)
{
    int dodaixe = 30;
    int distan = 15;

    if (xTam < xL)
    {
        if (  xTam > (xL - dodaixe/2 - distan) )
            xTam = xL + dodaixe/2 + distan;
        else
            xTam = xTam;
    }
    else if (xTam > xR)
    {
        if (  xTam < (xR + dodaixe/2 + distan) )
            xTam = xL + dodaixe/2 + distan;
        else
            xTam = xTam;
    }
    else
    {
	cout << "AAAAAAAAAAAAAAA";
    }
}

