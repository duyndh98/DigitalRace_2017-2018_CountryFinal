#ifndef __IMAGE_PROCESSING_H__
#define __IMAGE_PROCESSING_H__

#include "header.h"

void get_mask(const Mat &hsv, Mat &mask, string colors);
void hist_equalize(Mat &img);
Mat remOutlier(const Mat &gray);
char analyzeFrame(const VideoFrameRef &frame_depth, const VideoFrameRef &frame_color, Mat &depth_img, Mat &color_img);
double getTheta(Point car, Point dst);


#endif