#include "image_processing.h"

void get_mask(const Mat &hsv, Mat &mask, bool blue, bool red, bool black)
{
	mask = Mat::zeros(hsv.rows, hsv.cols, CV_8UC1);

	Mat tmp_mask(mask.rows, mask.cols, CV_8UC1);
	if (blue`)
	{
		inRange(hsv, LOW_HSV_BLUE, HIG_HSV_BLUE, tmp_mask);
		bitwise_or(mask, tmp_mask, mask);
	}
	if (red)
	{
		Mat tmp_mask1(mask.rows, mask.cols, CV_8UC1);
		Mat tmp_mask2(mask.rows, mask.cols, CV_8UC1);
		inRange(hsv, LOW_HSV_RED1, HIG_HSV_RED1, tmp_mask1);
		inRange(hsv, LOW_HSV_RED2, HIG_HSV_RED2, tmp_mask2);
		bitwise_or(tmp_mask1, tmp_mask2, tmp_mask);
		bitwise_or(mask, tmp_mask, mask);
	}
	if (black)
    {
        inRange(hsv, LOW_HSV_BLACK, HIG_HSV_BLACK, tmp_mask);
		bitwise_or(mask, tmp_mask, mask);
    }

	Mat kernel = Mat::ones(KERNEL_SIZE, KERNEL_SIZE, CV_8UC1);

	dilate(mask, mask, kernel);
	morphologyEx(mask, mask, MORPH_CLOSE, kernel);
}

// WARNING: should only be used once
void hist_equalize(Mat &img)
{
	Mat ycrcb;
	cvtColor(img, ycrcb, CV_BGR2YCrCb);

	vector<Mat> chanels(3);
	split(ycrcb, chanels);

	Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
	clahe->apply(chanels[0], chanels[0]);

	merge(chanels, ycrcb);
	cvtColor(ycrcb, img, CV_YCrCb2BGR);
}