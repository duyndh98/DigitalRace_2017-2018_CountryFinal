#include "lane_detection.h"
#include <stdio.h>
#include <stdlib.h>

Mat orgImg, colorImg, hsvImg, grayImg, binImg;
double theta, preTheta;
Point centerPoint(FRAME_WIDTH / 2, (1 - CENTER_POINT_Y) * FRAME_HEIGHT);
Point centerLeft(0, (1 - CENTER_POINT_Y) * FRAME_HEIGHT);
Point centerRight(0, (1 - CENTER_POINT_Y) * FRAME_HEIGHT);
Mat binLaneImg, colorLaneImg;
bool isLeft, isRight;

int findLargestContour(vector<vector<Point>> &contours)
{
    int i_max = -1;
    float maxArea = MIN_LANE_AREA;
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        int area = contourArea(contours[i]);
        if (area > maxArea)
        {
            i_max = i;
            maxArea = area;
        }
    }
    return i_max
}

// My function
void filterLane(Mat &binLaneImg, bool &isLane, int &centerX, int check)
{
    isLane = false;
    if (check < 0) // Left
    	centerX = 0;
	else // Right
        centerX = binLaneImg.cols;

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binLaneImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    int i_max = findLargestContour(contours);
    if (i_max != -1) // found !
    {
        isLane = true;
        Rect contourRect = boundingRect(contours[i_max]);
        if (check < 0) // Left
            centerX = contourRect.x + contourRect.width;
        else
            centerX = contourRect.x;
    }    
}

// Return angle between veritcal line containing car and destination point in degree
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

double getAngleLane() 
{
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;

    findContours(binLaneImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double maxArea = MIN_LANE_AREA;
    int i_max = -1;

    for (int i = 0; i < contours.size(); i++) 
    {
        double area = contourArea(contours[i]);
        if (maxArea < area) {
            maxArea = area;
            i_max = i;
        }
    }
    
    if (i_max == -1) 
        return preTheta; // not lane noisy
    
    Rect laneBound = boundingRect(contours[i_max]);
    rectangle(colorLaneImg, Point(laneBound.x, laneBound.y), Point(laneBound.x + laneBound.width, laneBound.y + laneBound.height), Scalar(255, 0, 0));    
   
    Point top(binImg.cols / 2, laneBound.y);
    Point bottom(binImg.cols / 2, laneBound.y + laneBound.height);
    
    for (int x = laneBound.x; x < laneBound.x + laneBound.width; x++)
    {
        if (binLaneImg.at<uchar>(laneBound.y, x) != 0) 
            top.x = x;
        if (binLaneImg.at<uchar>(laneBound.y + laneBound.height - 1, x) != 0) 
            bottom.x = x;
    }
     
    printf("top: (%d, %d)\n)", top.x, top.y);
    printf("bottom: (%d, %d)\n", bottom.x, bottom.y);
 
    return getTheta(bottom, top);
}

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst) 
{
	Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
	warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

void birdEye(Mat &binLaneImg, Mat &colorLaneImg)
{
	Point2f src_vertices[4];
	src_vertices[0] = Point((1 - RATIO_WIDTH_LANE_CROP) * 0.5 * binLaneImg.cols, 0);
	src_vertices[1] = Point((1 + RATIO_WIDTH_LANE_CROP) * 0.5 * binLaneImg.cols, 0);
	src_vertices[2] = Point(binLaneImg.cols, binLaneImg.rows);
	src_vertices[3] = Point(0, binLaneImg.rows);

	Point2f dst_vertices[4];
	dst_vertices[0] = Point(0, 0);
	dst_vertices[1] = Point(binLaneImg.cols, 0);
	dst_vertices[2] = Point(binLaneImg.cols, binLaneImg.rows);
	dst_vertices[3] = Point(0, binLaneImg.rows);

	Mat result(binLaneImg.rows, binLaneImg.cols, CV_8UC1);
	transform(src_vertices, dst_vertices, binLaneImg, result);
    result.copyTo(binLaneImg);

    line(colorLaneImg, src_vertices[0], src_vertices[1], Scalar(0, 0, 255), 3);
    line(colorLaneImg, src_vertices[1], src_vertices[2], Scalar(0, 0, 255), 3);
    line(colorLaneImg, src_vertices[2], src_vertices[3], Scalar(0, 0, 255), 3);
    line(colorLaneImg, src_vertices[3], src_vertices[0], Scalar(0, 0, 255), 3);
}

void LaneProcessing()
{   
    Rect laneRect(0, (1 - RATIO_HEIGHT_LANE_CROP) * FRAME_HEIGHT, 
                            FRAME_WIDTH, RATIO_HEIGHT_LANE_CROP * FRAME_HEIGHT);
    
    binLaneImg = binImg(laneRect);
    colorLaneImg = colorImg(laneRect);
    
    birdEye(binLaneImg, colorLaneImg);
    
    imshow("binLaneImg", binLaneImg);
    imshow("colorLaneImage", colorLaneImage);

    // Define rects to crop left and right windows from binary-lane after creating bird-view
    int xLeftRect = 0;
    int yLeftRect = 0;
    int xRightRect = (1 - RATIO_LEFT_RIGHT_WIDTH_LANE_CROP) * FRAME_WIDTH;
    int yRightRect = 0;
    int widthRect = RATIO_LEFT_RIGHT_WIDTH_LANE_CROP * FRAME_WIDTH;
    int heightRect = FRAME_HEIGHT;
    
    // Crop
    Rect rectLeft(xLeftRect, yLeftRect, widthRect, heightRect);
    Rect rectRight(xRightRect, yRightRect, widthRect, heightRect);
    Mat binLeft = binLaneImg(rectLeft);
    Mat binRight = binLaneImg(rectRight);

    // Filter lanes
    filterLane(binLeft, isLeft, centerLeft.x, -1);
    filterLane(binRight, isRight, centerRight.x, 1);
    centerLeft.x += xLeftRect;
    centerRight.x += xRightRect;
    
    imshow("Left", binLeft);
    imshow("Right", binRight);

    Point carPosition(FRAME_WIDTH / 2, FRAME_HEIGHT);
    
    if (!isLeft && !isRight)
    {
        putText(colorImg, "No lane", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
        theta = preTheta;
    }
    else if (!isLeft || !isRight || abs(int(centerLeft.x - centerRight.x)) < MIN_RATIO_DISTANCE_LEFT_RIGHT_CENTER * FRAME_WIDTH)
    {
        putText(colorImg, "Invalid distance", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);	
        theta = getAngleLane() * ALPHA;
    }
    else
    {
        putText(colorImg, "2 lane", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
        centerPoint.x = (centerLeft.x + centerRight.x) / 2;
        theta = getTheta(carPosition, centerPoint) * ALPHA;
    }
    
    // Backup
    preTheta = theta;
	
    // Draw center points
    circle(colorImg,  centerPoint, 2, Scalar(0, 0, 255), 2);
    line(colorImg, carPosition, centerPoint,Scalar(0, 0, 255), 2);
    
    putText(colorImg, "L", centerLeft, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 1, CV_AA);
    putText(colorImg, "R", centerRight, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 1, CV_AA);
    circle(colorImg, centerLeft, 2, Scalar(0, 255, 0), 2);
    circle(colorImg, centerRight, 2, Scalar(0, 255, 0), 2);

    if (theta > -10 && theta < 10)
	    theta = 0;
    
    putText(colorImg, "Theta " + to_string(int(theta)) , Point(0, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
    
    imshow("binLaneImg", binLaneImg);
    imshow("colorLaneImg", colorLaneImg);
}

void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img)
{
    color_img = Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
    RGB888Pixel *color_img_data = (RGB888Pixel *)frame_color.getData();
    memcpy(color_img.data, color_img_data, FRAME_HEIGHT * FRAME_WIDTH * sizeof(RGB888Pixel));
    cvtColor(color_img, color_img, COLOR_RGB2BGR);
}