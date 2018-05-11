#ifndef __DEPTH_PROCESSING_H__
#define __DEPTH_PROCESSING_H__

#include "header.h"
#include "depth_processing.h"

void
mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes,
                      cv::Mat &image,
                      std::vector<cv::Rect> &outputBoxes);
void
truncate(Mat &src, Mat &dst,
         int lower_bound, int upper_bound);
bool
api_kinect_cv_get_obtacle_rect( Mat& depthMap,
                                vector< Rect > &output_boxes,
                                Rect &roi,
                                int lower__bound,
                                int upper__bound, int thresh_area_min, int thresh_area_max);
void
api_kinect_cv_center_rect_gen(
        vector< Rect > &rects,
        int frame_width,
        int frame_height);
bool thoidiemre(Mat &depthMap);
int
api_kinect_cv_get_images(VideoCapture &capture,
        Mat &depthMap);


#endif