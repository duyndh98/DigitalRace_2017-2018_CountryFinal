#include "lane_detection.h"
#include <stdio.h>
#include <stdlib.h>

// My function
void filterLane(const Mat &imgLane, bool &isLine, int &centerX, int check)
{
    isLine = false;
    if (check < 0) // Left
    	centerX = 0;
	else // Right
        centerX = imgLane.cols;

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(imgLane, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    if (contours.size() == 0)
        return;
    
    Mat result = Mat::zeros(imgLane.size(), CV_8UC1);
    
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        int area = contourArea(contours[i]);
        if (area >= AREA_MIN)
        {
            drawContours(result, contours, i, Scalar(255), CV_FILLED);
            isLine = true;
            if (check < 0) // Left
            {
                // Find the most right vertice (xmax)
                for (int j = 0; j < contours[i].size(); ++j)
                    if (contours[i][j].x > centerX)
                        centerX = contours[i][j].x;
            }
            else // Right
            {
                for (int j = 0; j < contours[i].size(); ++j)
                {
                    // Find the most left vertice (xmin)
                    if (contours[i][j].x < centerX)
                        centerX = contours[i][j].x;
                }	
            }
        }
    }
    // Line found
    result.copyTo(imgLane);
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

double getAngleLane(Mat &binImg,double preTheta) 
{
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;

    findContours(binImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

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
    
    int x_top = binImg.cols/2;
    int x_bottom = binImg.cols/2;
    for (int x = laneBound.x; x < laneBound.x + laneBound.width; x++)
    {
        if (binImg.at<uchar>(laneBound.y + 1, x) != 0) 
        {
            x_top = x;
            //break;
        }
        if (binImg.at<uchar>(laneBound.y + laneBound.height - 1, x) != 0) 
        {
            x_bottom = x;
            //break;
        }
    }
	
	cout << "top: ( " << x_top << ", " << laneBound.y << ")" << endl;
	cout << "bottom ( " << x_bottom << ", " << laneBound.y + laneBound.height << ")" << endl;
 
	//circle(binImg, Point(x_top, laneBound.y), 2, Scalar(255, 255, 255), 10);
	//circle(binImg, Point(x_bottom, laneBound.y + laneBound.height), 2, Scalar(255, 255, 255), 10);
	imshow("lane", binImg);
	
  //  rectangle(binImg, Point(x_bottom, laneBound.y + laneBound.height), 3, Scalar(255,255,255));
    return getTheta(Point(x_bottom, laneBound.y + laneBound.height), Point(x_top, laneBound.y));
}

void LaneProcessing(Mat& colorImg, Mat& binImg, Point &centerPoint, Point &centerLeft, Point &centerRight, bool &isLeft, bool &isRight, double& theta, double &preTheta) 
{
    // Define rect to crop binImg into Left and Right
    int xLeftRect = 0;
    int yLeftRect = (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows;
    int xRightRect = (1.5 - RATIO_WIDTH_LANE_CROP) * binImg.cols;
    int yRightRect = (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows;
    int widthRect = RATIO_WIDTH_LANE_CROP * binImg.cols / 2;
    int heightRect = RATIO_HEIGHT_LANE_CROP * binImg.rows;
    
    // Crop
    Rect rectLeft(xLeftRect, yLeftRect, widthRect, heightRect);
    Rect rectRight(xRightRect, yRightRect, widthRect, heightRect);
    Mat binLeft = binImg(rectLeft);
    Mat binRight = binImg(rectRight);

    // Backup
    Point preCenterLeft = centerLeft;
    Point preCenterRight = centerRight;
    bool preIsLeft = isLeft;
    bool preIsRight = isRight;

    // Filter lanes
    filterLane(binLeft, isLeft, centerLeft.x, -1);
    filterLane(binRight, isRight, centerRight.x, 1);
    centerLeft.x += xLeftRect;
    centerRight.x += xRightRect;
    
    imshow("Left", binLeft);
    imshow("Right", binRight);

    Point carPosition(FRAME_WIDTH / 2, FRAME_HEIGHT);
    
    // if ((!isLeft && isRight) || (isLeft && !isRight)
    if (!isLeft || !isRight
        || (abs(int(centerLeft.x - centerRight.x)) < MIN_RATIO_DISTANCE_LEFT_RIGHT_CENTER * binImg.cols)) 
        //|| (abs(int(centerLeft.x - centerRight.x)) > MAX_RATIO_DISTANCE_LEFT_RIGHT_CENTER * binImg.cols)) 
        // Lost one lane
    {
        // // Shift center point
        // putText(colorImg, "Case 1", Point(0, 15), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
        // if (!isLeft) // Lost left lane
        // {
        //     putText(colorImg, "Lost left", Point(20, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
        //     centerLeft.x = preCenterLeft.x;// + centerRight.x - preCenterRight.x;
        //     centerRight.x = centerRight.x;
        //     centerPoint.x = (centerLeft.x + centerRight.x) / 2;
        //     theta = getTheta(carPosition, centerPoint);
        // }
        // else if(!isRight) // Lost right lane
        // {
        //     putText(colorImg, "Lost right", Point(20, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
        //     centerLeft = centerLeft;
        //     centerRight.x = preCenterRight.x;// + centerLeft.x - preCenterLeft.x;
        //     centerPoint.x = (centerLeft.x + centerRight.x) / 2;
        //     theta = getTheta(carPosition, centerPoint);
        // }
        // else // Has both lane but invalid -> get angle
        // {
            putText(colorImg, "Invalid distance", Point(20, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
            Mat laneImg = binImg(Rect(binImg.cols/6, (1 - 0.42) * binImg.rows, 
                               4*binImg.cols/6, 0.42 * binImg.rows));
	
            theta = getAngleLane(laneImg,preTheta);
        
    }
    // else if (!isLeft && !isRight) // Lost both lane
    // {
    //     putText(colorImg, "No left and right", Point(60, 100), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
    //     centerLeft.x = preCenterLeft.x;
    //     centerRight.x = preCenterRight.x;

    //     centerPoint.x = (centerLeft.x + centerRight.x) / 2;
    //     theta = getTheta(carPosition, centerPoint);
    // } // has both lane
    else
    {
        putText(colorImg, "Left Right", Point(60, 100), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
        centerPoint.x = (centerLeft.x + centerRight.x) / 2;
        theta = getTheta(carPosition, centerPoint);
    }
    
    // Draw center points
    circle(colorImg, centerPoint, 2, Scalar(255, 0, 0), 3);
    line(colorImg,carPosition,centerPoint,Scalar(255,0,0),3);
    putText(colorImg, "L", centerLeft, FONT_HERSHEY_COMPLEX_SMALL, 2.0, Scalar(0, 0, 255), 1, CV_AA);
    putText(colorImg, "R", centerRight, FONT_HERSHEY_COMPLEX_SMALL, 2.0, Scalar(0, 255, 0), 1, CV_AA);
    circle(colorImg, centerLeft, 2, Scalar(0, 0, 255), 3);
    circle(colorImg, centerRight, 2, Scalar(0, 255, 0), 3);
	preTheta = theta;
    theta = -theta * ALPHA;
    putText(colorImg, "theta " + to_string(int(theta)) , Point(0, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
	cout << "---------- Theta: " << theta << endl;
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

void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img)
{
    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    color_img = Mat(h, w, CV_8UC3);
    RGB888Pixel *color_img_data = (RGB888Pixel *)frame_color.getData();
    memcpy(color_img.data, color_img_data, h * w * sizeof(RGB888Pixel));
    cvtColor(color_img, color_img, COLOR_RGB2BGR);

    return;
}
