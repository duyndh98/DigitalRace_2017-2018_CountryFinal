#ifndef __SIGN_RECOGNIZER_H__
#define __SIGN_RECOGNIZER_H__

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/ml.hpp>
//#include <cv.h>

#include <string>
#include <iostream>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;
using namespace cv::ml;

class sign_recognizer {
    Mat _img;
    Mat _sign_mask;
    Rect _sign_rect;
    HOGDescriptor _hog;
    Ptr<SVM> _svm;

public:
    void init();

    void configure(const Mat& mask, const Mat& img, const Rect& sign);

    int recognize();    

private:
    int classify_sign(const Mat& sign_mask, const Mat& sign_gray);
};

#endif
