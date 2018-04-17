#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"
#include "image_processing.h"

cv::Mat filterLane(const cv::Mat &imgLane, Point &point, int check);
void LaneProcessing(Mat& colorImg, Mat& binImg, int &centerX, int &centerLeftX, int &centerRightX);
Mat remOutlier(const Mat &gray);
void analyzeFrame(/*const VideoFrameRef &frame_depth,*/ const VideoFrameRef &frame_color,/* Mat &depth_img,*/ Mat &color_img);
double getTheta(Point car, Point dst);

#endif
