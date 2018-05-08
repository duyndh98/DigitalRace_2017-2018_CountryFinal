#include "lane_detection.h"
#include <stdio.h>
#include <stdlib.h>

Mat colorImg, hsvImg, binImg;
double theta;

void filterLane(Mat &colorLaneImg, Mat binLaneImg, Point &centerLeft, Point &centerRight, bool &isLane)
{
    isLane = false;
    bool isLeft = false;
    bool isRight = false;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(binLaneImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (contours.size() == 0)
        return;

    binLaneImg = Mat::zeros(binLaneImg.size(), CV_8UC1);
    bool fLane = false;
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        int area = contourArea(contours[i]);
        if (area >= AREA_MIN)
        {
            double check = 0;
            Point pMin(binLaneImg.cols,0);
            Point pMax(0,0);
            drawContours(binLaneImg, contours, i, Scalar(255), CV_FILLED);
            Rect contourBoundingRect = boundingRect(contours[i]);
            Point p1(0,0);
            Point p2(0,0);
            int countUp = 0;
            int countDown = 0;
            for (int j = 0; j < contours[i].size(); ++j){
                if (contours[i][j].y < contourBoundingRect.y+contourBoundingRect.height/2){
                    p1.x += contours[i][j].x;
                    p1.y += contours[i][j].y;
                    countUp++;
                } else {
                    p2.x += contours[i][j].x;
                    p2.y += contours[i][j].y;
                    countDown++;
                }
                if(contours[i][j].x>pMax.x)
                    pMax = contours[i][j];
                if(contours[i][j].x<pMin.x)
                    pMin = contours[i][j];
            }
            p1.x = p1.x/countUp;
            p1.y = p1.y/countUp;
            p2.x = p2.x/countDown;
            p2.y = p2.y/countDown;
            double checkDistance = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
            if(checkDistance < MIN_DISTANCE)
                continue;
            //cout << "checkDistance: " << checkDistance << endl;
            line(colorLaneImg, p1, p2, Scalar(0, 255, 255), 3);
            circle(colorLaneImg, pMin, 2, Scalar(25, 255, 255), 3);
            circle(colorLaneImg, pMax, 2, Scalar(255, 0, 255), 3);
            double pA, pB;
            bool vg = false;
            if(p1.x==p2.x){
                vg = true;
                check = p1.x;
            } else {
                vg = false;
                pA = (double)(p1.y-p2.y)/(p1.x-p2.x);
                pB = (double)(p1.x*p2.y-p1.y*p2.x)/(p1.x-p2.x);
                check = (double)(binLaneImg.rows-pB)/pA;
                if(pA==0)
                    continue;
                //cout << "min(" << pMin.x << "," << pMin.y << "), max" << pMax.x  << pMax.y << ")" << endl;
                circle(colorLaneImg, Point(check,preCenterPoint.y), 2, Scalar(255, 255, 255), 3);
                //cout << "p1(" << p1.x << "," << p1.y << "),p2(" << p2.x << "," << p2.y<< ") check: " << check << endl;
            }
            if(check>preCenterPoint.x){
                isRight = true;
                if(vg){
                    check = pMin.x;
                } else {
                    pB = (double)pMin.y-pA*pMin.x;
                    check = (double)(preCenterPoint.y-pB)/pA;
                }
                if(centerRight.x>check){
                    centerRight.x = check;
                }
            } else {
                isLeft = true;
                if(vg){
                    check = pMax.x;
                } else {
                    pB = (double)pMax.y-pA*pMax.x;
                    check = (double)(preCenterPoint.y-pB)/pA;
                }
                if(centerLeft.x<check){
                    centerLeft.x = check;
                }
            }
            //cout << "check: " << check << endl;
	    fLane = true;
        }
    }
    if(!isLeft)
        centerLeft = preLeft;
    if(!isRight)
        centerRight = preRight;
    preLeft = centerLeft;
    preRight = centerRight;
    //cout << "center point: " << preCenterPoint.y << endl;
    if(!fLane)
    	isLane = false;
	else 
    isLane = true;
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

double getAngleLane(Mat &binImg, double preTheta)
{
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;

    findContours(binImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double maxArea = MIN_LANE_AREA;
    int i_max = -1;

    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (maxArea < area)
        {
            maxArea = area;
            i_max = i;
        }
    }

    if (i_max == -1)
        return preTheta; // not lane noisy

    Rect laneBound = boundingRect(contours[i_max]);

    Point top(binImg.cols / 2, laneBound.y);
    Point bottom(binImg.cols / 2, laneBound.y + laneBound.height);

    for (int x = laneBound.x; x < laneBound.x + laneBound.width; x++)
    {
        if (binImg.at<uchar>(laneBound.y, x) != 0)
            top.x = x;
        if (binImg.at<uchar>(laneBound.y + laneBound.height - 1, x) != 0)
            bottom.x = x;
    }

    printf("top: (%d, %d)\n)", top.x, top.y);
    printf("bottom: (%d, %d)\n", bottom.x, bottom.y);

    return getTheta(bottom, top);
}

void transform(Point2f *src_vertices, Point2f *dst_vertices, Mat &src, Mat &dst)
{
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

void cropBirdEye(Mat &binLaneImg, Mat &colorLaneImg)
{
    Point2f src_vertices[4];
    src_vertices[0] = Point((1 - RATIO_WIDTH_LANE_CROP) * 0.5 * binLaneImg.cols, 0);
    src_vertices[1] = Point((1 + RATIO_WIDTH_LANE_CROP) * 0.5 * binLaneImg.cols, 0);
    src_vertices[2] = Point(binLaneImg.cols, 3 * binLaneImg.rows / 4);
    src_vertices[3] = Point(0, 3 * binLaneImg.rows / 4);

    Point2f dst_vertices[5];
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

void laneProcessing()
{
	cout << "preCenterPoint: " << preCenterPoint.x << " " << preCenterPoint.y << endl;
    bool isLane;

    //bool isLeft = true, isRight = true;

    Rect laneRect(0, (1 - RATIO_HEIGHT_LANE_CROP) * binImg.rows,
        binImg.cols, RATIO_HEIGHT_LANE_CROP * binImg.rows);

    Mat binLaneImg = binImg(laneRect);
    Mat colorLaneImg = colorImg(laneRect);

    //cropBirdEye(binLaneImg, colorLaneImg);
    Point centerPoint(binLaneImg.cols / 2, (1 - CENTER_POINT_Y) * binLaneImg.rows);
    Point centerLeft(0, (1 - CENTER_POINT_Y) * binLaneImg.rows);
    Point centerRight(binLaneImg.cols, (1 - CENTER_POINT_Y) * binLaneImg.rows);

    // Define rects to crop left and right windows from binary-lane after creating bird-view
    // int xLeftRect = 0;
    // int yLeftRect = 0;
    // int xRightRect = (1 - RATIO_LEFT_RIGHT_WIDTH_LANE_CROP) * binLaneImg.cols;
    // int yRightRect = 0;
    // int widthRect = RATIO_LEFT_RIGHT_WIDTH_LANE_CROP * binLaneImg.cols;
    // int heightRect = binLaneImg.rows;

    // Crop
    // Rect rectLeft(xLeftRect, yLeftRect, widthRect, heightRect);
    // Rect rectRight(xRightRect, yRightRect, widthRect, heightRect);
    // Mat binLeft = binLaneImg(rectLeft);
    // Mat binRight = binLaneImg(rectRight);

    // Backup
    // Point preCenterLeft = centerLeft;
    // Point preCenterRight = centerRight;

    // bool preIsLeft = isLeft;
    // bool preIsRight = isRight;
    //double preTheta = theta;

    // Filter lanes
    // filterLane(binLeft, isLeft, centerLeft.x, -1);
    // filterLane(binRight, isRight, centerRight.x, 1);
    filterLane(colorLaneImg, binLaneImg, centerLeft, centerRight, isLane);
    if (isDebug)
    {
        imshow("binLaneImg", binLaneImg);
        imshow("colorLaneImg", colorLaneImg);
    }
    // centerLeft.x += xLeftRect;
    // centerRight.x += xRightRect;

    // imshow("Left", binLeft);
    // imshow("Right", binRight);

    // if (!isLeft && !isRight)
    // {
    //     putText(colorImg, "No lane", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
    //     theta = 0;
    // }
    // else if (abs(int(centerLeft.x - centerRight.x)) < MIN_RATIO_DISTANCE_LEFT_RIGHT_CENTER * binLaneImg.cols)
    // {
    //     putText(colorImg, "Invalid distance", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
    //     theta = getAngleLane(binLaneImg, preTheta);
    // }
    // else
    // {
    //     if (!isLeft)
    //     {
    //         putText(colorImg, "Lost left", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
    //         centerLeft = preCenterLeft + centerRight - preCenterRight;
    //     }
    //     else if (!isRight)
    //     {
    //         putText(colorImg, "Lost right", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);
    //         centerRight= preCenterRight + centerLeft - preCenterLeft;
    //     }
    //     else
    //         putText(colorImg, "2 lane", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 1, CV_AA);

    //     centerPoint.x = (centerLeft.x + centerRight.x) / 2;
    //     theta = getTheta(carPosition, centerPoint);
    // }
    if (isLane)
    {
        if (isDebug)
            putText(colorImg, "Has Lane", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 0), 1, CV_AA);
        centerPoint.x = (centerLeft.x + centerRight.x) / 2;
        centerPoint.y = (centerLeft.y + centerRight.y) / 2 + colorImg.rows*RATIO_HEIGHT_LANE_CROP;
        circle(colorImg, centerLeft, 2, Scalar(0, 0, 255), 3);
        circle(colorImg, centerRight, 2, Scalar(0, 0, 255), 3);
        cout << "left: " << centerLeft.x << " right: " << centerRight.x << endl;
    }
    else
    {
        if (isDebug)
        putText(colorImg, "No Lane", Point(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 0), 1, CV_AA);
        centerPoint.x = preCenterPoint.x;
        centerPoint.y = preCenterPoint.y + colorImg.rows*RATIO_HEIGHT_LANE_CROP;
    }

    theta = getTheta(carPosition, centerPoint);

    // Draw center points
    circle(colorImg, centerPoint, 2, Scalar(0, 0, 255), 3);
    line(colorImg, carPosition, centerPoint, Scalar(0, 0, 255), 3);

    // putText(colorImg, "L", centerLeft, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0, 0), 1, CV_AA);
    // putText(colorImg, "R", centerRight, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 1, CV_AA);

    theta = -theta * ALPHA;
    if (theta > -20 && theta < 20)
        theta = 0;

    if (isDebug)
        putText(colorImg, "Theta " + to_string(int(theta)), Point(0, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 0), 1, CV_AA);
    
    printf("theta: %d\n", int(theta));
	preCenterPoint.x = centerPoint.x;
    preCenterPoint.y = centerPoint.y - colorImg.rows*RATIO_HEIGHT_LANE_CROP;
}

void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img)
{
    color_img = Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
    RGB888Pixel *color_img_data = (RGB888Pixel *)frame_color.getData();
    memcpy(color_img.data, color_img_data, FRAME_HEIGHT * FRAME_WIDTH * sizeof(RGB888Pixel));
    cvtColor(color_img, color_img, COLOR_RGB2BGR);
}
