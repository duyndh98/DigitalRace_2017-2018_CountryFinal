#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"
#include "image_processing.h"

void filterLane(Mat &colorLaneImg, Mat binLaneImg, Point &centerLeft, Point &centerRight, bool &isLane);
double getTheta(Point car, Point dst);
double getAngleLane(Mat &binImg, double preTheta);
void transform(Point* src_vertices, Point* dst_vertices, Mat& src, Mat &dst);
void cropBirdEye(Mat &binLaneImg, Mat &colorLaneImg);
void laneProcessing();
void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img);
void remOutlier(Mat &gray);

#endif
