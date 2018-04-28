#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"
#include "image_processing.h"

int findLargestContour(vector<vector<Point>> &contours);
void filterLane(Mat &binLaneImg, bool &isLine, int &centerX, int check);
double getTheta(Point car, Point dst);
double getAngleLane();
void transform(Point* src_vertices, Point* dst_vertices, Mat& src, Mat &dst);
void birdEye();
void laneProcessing();
void analyzeFrame(const VideoFrameRef &frame_color, Mat &color_img);
void remOutlier(Mat &gray);

#endif
