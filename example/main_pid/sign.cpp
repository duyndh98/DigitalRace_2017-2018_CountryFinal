#include "sign.h"

Sign::Sign()
{
	_hog = HOGDescriptor(Size(SIGN_SIZE, SIGN_SIZE),
		Size(SIGN_SIZE / 2, SIGN_SIZE / 2),
		Size(SIGN_SIZE / 4, SIGN_SIZE / 4),
		Size(SIGN_SIZE / 4, SIGN_SIZE / 4),
		9
		);

	_svm = SVM::load("svm_model.xml");
	_sign_ROI = Rect(0, 0, 0, 0);
	_class_id = 0;
}

bool Sign::detect()
{
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(binSignImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));

	// set default is no sign found
	double max_area = 0;
	_sign_ROI = Rect(0, 0, 0, 0);

	for (int i = 0; i < contours.size(); i++)
	{
		Rect bound = boundingRect(contours[i]);
		double contour_area = contourArea(contours[i]);
		if (contour_area <= max_area)
			continue;

		// constraints
		double ellipse_area = (3.14f * (double)(bound.width / 2) * (double)(bound.height / 2));
		if (contour_area >= MIN_SIGN_AREA)
			if ((1 - DIF_RATIO_SIGN_WIDTH_PER_HEIGHT < (float)bound.width / bound.height) && ((float)bound.width / bound.height < 1 + DIF_RATIO_SIGN_WIDTH_PER_HEIGHT))
				if ((1 - DIF_RATIO_SIGN_AREA < ((double)contour_area / ellipse_area)) && ((double)contour_area / ellipse_area < 1 + DIF_RATIO_SIGN_AREA))
				{
					// update max sign
					_sign_ROI = bound;
					max_area = contour_area;
				}
	}
	return _sign_ROI != Rect(0, 0, 0, 0);
}

void Sign::recognize()
{
	// no sign
	if (_sign_ROI == Rect(0, 0, 0, 0))
	{
		_class_id = NO_SIGN;
	}
	// crop
	Mat sign_gray = grayImg(_sign_ROI);
	resize(sign_gray, sign_gray, Size(SIGN_SIZE, SIGN_SIZE));

	classify(sign_gray);
}

void Sign::classify(Mat &graySignImg)
{
	//imshow("sign_gray", sign_gray);
	// compute HOG descriptor
	vector<float> descriptors;
	_hog.compute(graySignImg, descriptors);
	
	Mat fm(descriptors, CV_32F);
	// predict matrix transposition
	_class_id = (int)(_svm->predict(fm.t()));
	if (_class_id != SIGN_LEFT && _class_id != SIGN_RIGHT && _class_id != SIGN_STOP)
	{
		_class_id = NO_SIGN;
		_sign_ROI = Rect(0, 0, 0, 0);
	}
}

int Sign::getClassID()
{
	return _class_id;
}

Rect Sign::getROI()
{
	return _sign_ROI;
}