#include "lane_detection.h"

/////// My function
Mat filterLane(const Mat &imgLane, bool &pop, Point &point, int check, bool &preState)
{
    pop = false;
    if (check==-1){
		point.x = 0;
		point.y = imgLane.rows/2;
	} else {
		point.x = imgLane.cols;
		point.y = imgLane.rows/2;	
	}
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(imgLane, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
    if (contours.size() == 0)
    {
        Mat none = Mat::zeros(imgLane.size(), CV_8UC1);
        return none;
    }
    Mat result = Mat::zeros(imgLane.size(), CV_8UC1);
    int sumX = 0;
    int sumY = 0;
    int maxArea = 0;
    int maxIndex = 0;
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        int s = contourArea(contours[i]);
        if (s > maxArea)
        {
            maxArea = s;
            maxIndex = i;
        }
    }
    if (contourArea(contours[maxIndex]) < 100)
    {
        Mat none = Mat::zeros(imgLane.size(), CV_8UC1);
        return none;
    }
    int xMin = 0, yMin = 1000, xMax = 0, yMax = -1;
    drawContours(result, contours, maxIndex, Scalar(255), CV_FILLED);
    if (check == -1)
    {
		point.x = 0;
		for (int i = 0; i < contours[maxIndex].size(); i++)
        {
            if (point.x < contours[maxIndex][i].x)
                point.x = contours[maxIndex][i].x;
        }	
	} 
    else 
    {
		point.x = imgLane.cols;
		for (int i = 0; i < contours[maxIndex].size(); i++)
        {
            if (point.x > contours[maxIndex][i].x)
                point.x = contours[maxIndex][i].x;
        }	
	}
	point.y = imgLane.rows/2;
    pop = true;
    return result;
}


void LaneProcessing(Mat& colorImg, Mat& binImg, int& xTam, int& yTam, bool& preLeft, bool& preRight, bool& oneLine,int preX,int preY, int &preLeftX, int &preRightX) 
{
    int xLeft = 0;
    int yLeft = (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows;
    int xRight = (0.5 + 1 - RATIO_WIDTH_LANE_CROP) * binImg.cols;
    int yRight = (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows;
    
    Rect cropLeft(xLeft, yLeft, RATIO_WIDTH_LANE_CROP * binImg.cols / 2, RATIO_HEIGHT_LANE_CROP * binImg.rows);
    Rect cropRight(xRight, yRight, RATIO_WIDTH_LANE_CROP * binImg.cols / 2, RATIO_HEIGHT_LANE_CROP * binImg.rows);
    Mat Left = binImg(cropLeft);
    Mat Right = binImg(cropRight);
    
    Mat dstLeft = keepLanes(Left, false);
    Mat dstRight = keepLanes(Right, false);
    bool isLeft = false;
    bool isRight = false;
    Point pointLeft(0, 0);
    Point pointRight(0, 0);
    
    Left = filterLane(dstLeft, isLeft, pointLeft, -1, preLeft);
    Right = filterLane(dstRight, isRight, pointRight, 1, preRight);
    
    if (isLeft)
        pointLeft.x += xLeft;
    else // Lose center point => get the previous
        pointLeft.x = preLeftX;
    pointLeft.y += yLeft;
            
    if (isRight)
        pointRight.x += xRight;
    else // Lose center point => get the previous
        pointRight.x = preRightX;
    pointRight.y += yRight;
    
    // Backup
    preLeft = isLeft;
    preRight = isRight;
    preLeftX = pointLeft.x;
    preRightX = pointRight.x;
    imshow("LEFT", Left);
    imshow("RIGHT", Right);
    
    cout << "Left: " << isLeft << " Right: " << isRight << endl;
    
    xTam = (pointLeft.x + pointRight.x) / 2;
    yTam = (pointLeft.y + pointRight.y) / 2;
    
    // Draw center points
    circle(colorImg, Point(xTam, yTam), 2, Scalar(255, 255, 0), 3);
    circle(colorImg, Point(pointLeft.x, pointLeft.y), 2, Scalar(255, 255, 0), 3);
    circle(colorImg, Point(pointRight.x, pointRight.y), 2, Scalar(255, 255, 0), 3);
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
double getTheta(Point car, Point dst)
{
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
