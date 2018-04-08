#include "sign_recognizer.h"

#define SIGN_SIZE 48

void sign_recognizer::init() {
	_hog = HOGDescriptor(Size(SIGN_SIZE, SIGN_SIZE),
					Size(SIGN_SIZE / 2, SIGN_SIZE / 2),
					Size(SIGN_SIZE / 4, SIGN_SIZE / 4),
					Size(SIGN_SIZE / 2, SIGN_SIZE / 2),
					9,
					1,
					-1,
					0,
					0.2,
					1,
					64,
					true);

	_svm = SVM::load("svm_model.xml");

}

void sign_recognizer::configure(const Mat& mask, const Mat& img, const Rect& sign) {
    _sign_mask = mask;
	_img = img;
	_sign_rect = sign;
}

int sign_recognizer::recognize() {
	if (_sign_rect.x == 0 && _sign_rect.y == 0 && _sign_rect.width == 0 && _sign_rect.height == 0)
	{
		return 0;
	}

	// imshow("mask", mask);
	Mat sign_img = _img(_sign_rect);
	resize(sign_img, sign_img, cv::Size(SIGN_SIZE, SIGN_SIZE));
	imshow("sign", sign_img);

	Mat sign_gray;
	cvtColor(sign_img, sign_gray, COLOR_BGR2GRAY);

	Mat sign_mask = _sign_mask(_sign_rect);
	resize(sign_mask, sign_mask, cv::Size(SIGN_SIZE, SIGN_SIZE));

	return classify_sign(sign_mask, sign_gray);
}

int sign_recognizer::classify_sign(const Mat& sign_mask, const Mat& sign_gray)
{
	//HOGDescriptor hog(Size(32, 16), Size(8, 8), Size(4, 4), Size(4, 4), 9);
	vector<float> descriptors;
	vector<Point> locations;
	_hog.compute(sign_gray, descriptors);//, Size(SIGN_SIZE, SIGN_SIZE), Size(0, 0), locations);
	//Mat fm(descriptors.size(), descriptors.size(), CV_32FC1);
	Mat fm(descriptors, CV_32F);
	return (int)(_svm->predict(fm.t()));
}
