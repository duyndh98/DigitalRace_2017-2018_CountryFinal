void get_obtacle(Mat depthMap, int &xL, int &xR, int &y, int &w, int &h)
{
     // Init//
    int slice_nb = 3;
    int lower_slice_idx = 3;
    int upper_slice_idx = lower_slice_idx + slice_nb;
    //int lower_bound = DIST_MIN + lower_slice_idx * SLICE_DEPTH;
    //int upper_bound = lower_bound + slice_nb*SLICE_DEPTH;
    int lower_bound = 30;
    int upper_bound = 50;

    Mat depthMap;
    Mat grayImage;
    vector< Rect > rects;
    Rect intersect;
    vector< Rect > output_boxes;

    Rect roi_1 = Rect(0, VIDEO_FRAME_HEIGHT/4,
                    VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT/2);
    Rect roi_2(0, 132, 640, 238);

    int frame_width = capture.get( CV_CAP_PROP_FRAME_WIDTH );
    int frame_height = capture.get( CV_CAP_PROP_FRAME_HEIGHT );

    //api_kinect_cv_get_images(capture, depthMap , grayImage); //get depth

    crop_grayImage = grayImage(roi_1);
    crop_depthMap = depthMap(roi_1);

    api_kinect_cv_get_obtacle_rect( depthMap, output_boxes, roi_2, lower_bound, upper_bound );
    Mat binImg = Mat::zeros(depthMap.size(), CV_8UC1);

    api_kinect_cv_center_rect_gen( rects, frame_width, frame_height);
    Rect center_rect = rects[lower_slice_idx];
    center_rect = center_rect + Size(0, slice_nb*SLICE_DEPTH); // lay center_rect

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
        //lay toa do, rong, cao.
    xL = intersect.x;
    xR = intersect.x + intersect.width;
    y = intersect.y;
    w = intersect.width;
    h = intersect.height;
	cout<< "(x,y) = " << xL << ";"<< y <<endl << "width: "<< intersect.width<< endl << "height" <<intersect.height <<endl << "~~~~~~~~~~~~~~~~~~~~~";

    imshow( "BoundingRect", binImg );
    if(!grayImage.empty())
        imshow( "gray", crop_grayImage );

    if (!depthMap.empty() )
        imshow( "depth", crop_depthMap );
}
void PointCenter_Displacement(int &xTam, int xL, int xR)
{
    int dodaixe = 30;
    int distan = 7;

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

    }
}
void Delay( int   )
{
 
}
