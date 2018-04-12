#ifndef __IMAGE_PROCESSING_H__
#define __IMAGE_PROCESSING_H__

#include "header.h"
#include "image_processing.h"

void get_mask(const Mat &hsv, Mat &mask, string colors);
void hist_equalize(Mat &img);


#endif