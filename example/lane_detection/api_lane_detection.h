#include "stdio.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"

#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include "msac/MSAC.h"

using namespace std;
using namespace cv;

#define MAX_NUM_LINES	30
#define VIDEO_FRAME_WIDTH 320
#define VIDEO_FRAME_HEIGHT 240
#define debug false


#define SET_FAR_LANE 100
//width and height 
#define VIDEO_WIDTH_FRAME 320
#define VIDEO_HEIGHT_FRAME 240


//set region lane
#define LEFT_UP Point(20,120)
#define LEFT_DOWN Point(0,240)
#define RIGHT_UP Point(300,120)
#define RIGHT_DOWN Point(320,240)
//Canny edge index
/*
#define TH_LOWER_EDGE 100
#define RATIO 3
#define KERNEL_SIZE 3
*/
//set houghlineP
#define RHO 10
#define THREH 100
#define MIN_LENTH 100
#define MAX_GAP 100

using namespace std;
using namespace cv;

class Lane_Detector
{
private:
	int xc, yc;
public:
	Lane_Detector();
	~Lane_Detector();

	void Detect_Not_Lane(Mat,Point&);
	void Detect_White_Lane(Mat&,Point&);
	void Process_Lane_binImg(Mat&,Point&);
	Point getPoint();
};


///////////-----------------------------------------

void
api_vanishing_point_init(MSAC &msac);

void waveletTransform(const cv::Mat& img, cv::Mat& edge, double threshold);

void edgeProcessing(Mat org, Mat &dst, Mat element, string method);
void
api_get_vanishing_point(Mat imgGray, // input
                        Rect roi,    // input
                        MSAC &msac,  // input
                        Point &vp,    // output vnishing point
                        bool is_show_output,
                        string method);
                    //Current supported method: Canny, Sobel, Prewitt, Roberts
int
laneDetect(
        cv::Mat imgGrayOrigin,
        cv::Rect rectDetect,
        vector<vector<cv::Point> > &lineSegments,
        string method
        );
        

void
api_get_lane_center(Mat &imgGray,
                    Point &center_point,
                    bool is_show_output);
