#include "image_processing.h"

void get_mask(const Mat &hsv, Mat &mask, string colors)
{
	mask = Mat::zeros(hsv.rows, hsv.cols, CV_8UC1);

	Mat tmp_mask(mask.rows, mask.cols, CV_8UC1);
	if (colors.find("blue") != std::string::npos)
	{
		inRange(hsv, LOW_HSV_BLUE, HIG_HSV_BLUE, tmp_mask);
		bitwise_or(mask, tmp_mask, mask);
	}
	if (colors.find("red") != std::string::npos)
	{
		Mat tmp_mask1(mask.rows, mask.cols, CV_8UC1);
		Mat tmp_mask2(mask.rows, mask.cols, CV_8UC1);
		inRange(hsv, LOW_HSV_RED1, HIG_HSV_RED1, tmp_mask1);
		inRange(hsv, LOW_HSV_RED2, HIG_HSV_RED2, tmp_mask2);
		bitwise_or(tmp_mask1, tmp_mask2, tmp_mask);
		bitwise_or(mask, tmp_mask, mask);
	}
	if (colors.find("green") != std::string::npos)
	{
		inRange(hsv, LOW_HSV_GREEN, HIG_HSV_GREEN, tmp_mask);
		bitwise_or(mask, tmp_mask, mask);
	}
    if (colors.find("black") != std::string::npos)
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

Mat remOutlier(const Mat &gray)
{
    int esize = 1;
    Mat element = getStructuringElement(MORPH_RECT,
                                                Size(2 * esize + 1, 2 * esize + 1),
                                                Point(esize, esize));
    erode(gray, gray, element);
    std::vector<std::vector<Point>> contours, polygons;
    std::vector<Vec4i> hierarchy;
    findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    for (size_t i = 0; i < contours.size(); ++i)
    {
        std::vector<Point> p;
        approxPolyDP(Mat(contours[i]), p, 2, true);
        polygons.push_back(p);
    }
    Mat poly = Mat::zeros(gray.size(), CV_8UC3);
    for (size_t i = 0; i < polygons.size(); ++i)
    {
        Scalar color = Scalar(255, 255, 255);
        drawContours(poly, polygons, i, color, CV_FILLED);
    }
    return poly;
}

char analyzeFrame(const VideoFrameRef &frame_depth, const VideoFrameRef &frame_color, Mat &depth_img, Mat &color_img)
{
    DepthPixel *depth_img_data;
    RGB888Pixel *color_img_data;

    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    depth_img = Mat(h, w, CV_16U); ////////////////--------------------------------------------test
    color_img = Mat(h, w, CV_8UC3);
    Mat depth_img_8u;

    depth_img_data = (DepthPixel *)frame_depth.getData();

    memcpy(depth_img.data, depth_img_data, h * w * sizeof(DepthPixel));

    normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);

    depth_img_8u.convertTo(depth_img_8u, CV_8U);
    color_img_data = (RGB888Pixel *)frame_color.getData();

    memcpy(color_img.data, color_img_data, h * w * sizeof(RGB888Pixel));

    cvtColor(color_img, color_img, COLOR_RGB2BGR);

    return 'c';
}

/// Return angle between veritcal line containing car and destination point in degree
double getTheta(Point car, Point dst)
{
    if (dst.x == car.x)
        return 0;
    if (dst.y == car.y)
        return (dst.x < car.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
    if (dx < 0)
        return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}
