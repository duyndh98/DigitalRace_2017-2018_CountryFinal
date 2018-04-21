#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"
#include "image_processing.h"

double getTheta(Point car, Point dst);
double getAngleLane(Mat &binImg, double preTheta);
void filterLane(const Mat &imgLane, bool &isLine, int &centerX, int check);
void LaneProcessing(Mat& colorImg, Mat& binImg, Point &centerPoint, Point &centerLeft, Point &centerRight, bool &isLeft, bool &isRight, double& theta,  double &preTheta); 
Mat remOutlier(const Mat &gray);
void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img);
#endif
