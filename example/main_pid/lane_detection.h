#ifndef __LANE_DETECTION_H__
#define __LANE_DETECTION_H__

#include "header.h"

cv::Mat filterLane(const cv::Mat &imgLane, bool &pop, Point &point, int check, bool &preState);

#endif