#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"
#include "image_processing.h"

cv::Mat filterLane(const cv::Mat &imgLane, bool &pop, Point &point, int check, bool &preState);
void LaneProcessing(cv::Mat& colorImg,cv::Mat& binImg,int& xTam,int& yTam,bool& preLeft,bool& preRight,bool& oneLine,int preX,int preY);
Mat remOutlier(const Mat &gray);
char analyzeFrame(const VideoFrameRef &frame_depth, const VideoFrameRef &frame_color, Mat &depth_img, Mat &color_img);
double getTheta(Point car, Point dst);

#endif