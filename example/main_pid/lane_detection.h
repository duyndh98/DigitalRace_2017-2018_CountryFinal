#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"
#include "image_processing.h"

void filterLane(const cv::Mat &imgLane, Point &point, int check);
void LaneProcessing(Mat& colorImg, Mat& binImg, Point &centerPoint, Point &centerLeft, Point &centerRight);
Mat remOutlier(const Mat &gray);
void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img);
double getTheta(Point car, Point dst);

#endif
