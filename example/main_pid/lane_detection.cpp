#include "lane_detection.h"

// My function
void filterLane(const Mat &imgLane, bool &isLine, int &centerX, int check)
{
    isLine = false;
    if (check == -1) // Left
    	centerX = 0;
	else // Right
        centerX = imgLane.cols;

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(imgLane, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
    
    if (contours.size() == 0)
        return;
    
    Mat result = Mat::zeros(imgLane.size(), CV_8UC1);
    int area_max = 0;
    int i_max = 0;
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        int area = contourArea(contours[i]);
        if (area > area_max)
        {
            area_max = area;
            i_max = i;
        }
    }
    
    // No line found
    if (contourArea(contours[i_max]) < AREA_MIN)
        return;
    
    drawContours(result, contours, i_max, Scalar(255), CV_FILLED);
    
    if (check == -1) // Left
    {
		centerX = 0;
        // Browse through all of max-contour vertices
		for (int i = 0; i < contours[i_max].size(); i++)
        {
            // Find the most right vertice (xmax)
            if (centerX < contours[i_max][i].x)
                centerX = contours[i_max][i].x;
        }	
	}
    else // Right
    {
		centerX = imgLane.cols;
		for (int i = 0; i < contours[i_max].size(); i++)
        {
            // Find the most left vertice (xmin)
            if (centerX > contours[i_max][i].x)
                centerX = contours[i_max][i].x;
        }	
	}
	
    // Line found
    isLine = true;
}

void LaneProcessing(Mat& colorImg, Mat& binImg, Point &centerPoint, Point &centerLeft, Point &centerRight) 
{
    // Define rect to crop binImg into Left and Right
    int xLeftRect = 0;
    int yLeftRect = (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows;
    int xRightRect = (0.5 + 1 - RATIO_WIDTH_LANE_CROP) * binImg.cols;
    int yRightRect = (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows;
    int widthRect = RATIO_WIDTH_LANE_CROP * binImg.cols / 2;
    int heightRect = RATIO_HEIGHT_LANE_CROP * binImg.rows;
    
    Rect rectLeft(xLeftRect, yLeftRect, widthRect, heightRect);
    Rect rectRight(xRightRect, yRightRect, widthRect, heightRect);
    Mat binLeft = binImg(rectLeft);
    Mat binRight = binImg(rectRight);
    
    // Keep lanes
    binLeft = keepLanes(binLeft, false);
    binRight = keepLanes(binRight, false);

    bool isLeft = false;
    bool isRight = false;
    
    // Filter lanes
    Point preCenterLeft = centerLeft;
    Point preCenterRight = centerRight;
    filterLane(binLeft, isLeft, centerLeft.x, -1);
    filterLane(binRight, isRight, centerRight.x, 1);
    
    imshow("LEFT", binLeft);
    imshow("RIGHT", binRight);
    cout << "Left: " << isLeft << " Right: " << isRight << endl;
    
    if (isLeft)
        centerLeft.x += xLeftRect;
    else // Lose center point => get the previous
        centerLeft.x = preCenterLeft.x;
            
    if (isRight)
        centerRight.x += xRightRect;
    else // Lose center point => get the previous
        centerRight.x = preCenterRight.x;
    
    // Backup
    centerPoint.x = (centerLeft.x + centerRight.x) / 2;
    
    // Draw center points
    circle(colorImg, centerPoint, 2, Scalar(255, 255, 0), 3);
    circle(colorImg, centerLeft, 2, Scalar(255, 255, 0), 3);
    circle(colorImg, centerRight, 2, Scalar(255, 255, 0), 3);
    imshow("color", colorImg);
}

Mat remOutlier(const Mat &gray)
{
    int esize = 1;
    Mat element = getStructuringElement(MORPH_RECT,
                                                Size(2 * esize + 1, 2 * esize + 1),
                                                Point(esize, esize));
    erode(gray, gray, element);
    std::vector<std::vector<Point>> contours, polygons;
    std::vector<Vec4i> hierarchy;
    findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    for (size_t i = 0; i < contours.size(); ++i)
    {
        std::vector<Point> p;
        approxPolyDP(Mat(contours[i]), p, 2, true);
        polygons.push_back(p);
    }
    Mat poly = Mat::zeros(gray.size(), CV_8UC3);
    for (size_t i = 0; i < polygons.size(); ++i)
    {
        Scalar color = Scalar(255, 255, 255);
        drawContours(poly, polygons, i, color, CV_FILLED);
    }
    return poly;
}

void analyzeFrame(/*const VideoFrameRef &frame_depth,*/ const VideoFrameRef &frame_color,/* Mat &depth_img,*/ Mat &color_img)
{
    //DepthPixel *depth_img_data;
    RGB888Pixel *color_img_data;

    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    color_img = Mat(h, w, CV_8UC3);
    //depth_img = Mat(h, w, CV_16U); ////////////////--------------------------------------------test
    //Mat depth_img_8u;
    //depth_img_data = (DepthPixel *)frame_depth.getData();
    //memcpy(depth_img.data, depth_img_data, h * w * sizeof(DepthPixel));
    //normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
    //depth_img_8u.convertTo(depth_img_8u, CV_8U);

    color_img_data = (RGB888Pixel *)frame_color.getData();

    memcpy(color_img.data, color_img_data, h * w * sizeof(RGB888Pixel));

    cvtColor(color_img, color_img, COLOR_RGB2BGR);
    return;
}

/// Return angle between veritcal line containing car and destination point in degree
double getTheta(Point dst)
{
    Point car(0, 0);
    if (dst.x == car.x)
        return 0;
    if (dst.y == car.y)
        return (dst.x < car.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
    if (dx < 0)
        return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}
