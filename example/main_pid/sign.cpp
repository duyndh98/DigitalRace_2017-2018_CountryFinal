#include "sign.h"

Mat binBlueImg, binRedImg;

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

bool Sign::detect(bool blueSign)
{
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	if (blueSign)
		findContours(binBlueImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	else
		findContours(binRedImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// set default is no sign found
	double max_area = MIN_SIGN_AREA;
	_sign_ROI = Rect(0, 0, 0, 0);

	for (int i = 0; i < contours.size(); i++)
	{
		double contour_area = contourArea(contours[i]);
		if (contour_area < max_area)
			continue;

		Rect bound = boundingRect(contours[i]);
		// constraints
		double ellipse_area = (3.14f * (double)(bound.width / 2) * (double)(bound.height / 2));
		if ((1 - DIF_RATIO_SIGN_WIDTH_PER_HEIGHT < (float)bound.width / bound.height) && ((float)bound.width / bound.height < 1 + DIF_RATIO_SIGN_WIDTH_PER_HEIGHT))
			if ((1 - DIF_RATIO_SIGN_AREA < ((double)contour_area / ellipse_area)) && ((double)contour_area / ellipse_area < 1 + DIF_RATIO_SIGN_AREA))
			{
				// update max sign
				_sign_ROI = bound;
				max_area = contour_area;
			}
	}

	// Stop sign case
	if (_sign_ROI == Rect(0, 0, 0, 0) && !blueSign)
	{
		double area_max1 = MIN_SIGN_AREA;
		double area_max2 = MIN_SIGN_AREA;
		int i_max1 = -1, i_max2 = -1;
		
		for (int i = 0; i < contours.size(); i++)
		{
			double contour_area = contourArea(contours[i]);
			if (contour_area < MIN_SIGN_AREA)
				continue;

			Rect bound = boundingRect(contours[i]);
			// update max sign
			if (contour_area > area_max1)
			{
				area_max2 = area_max1;
				i_max2 = i_max1;

				area_max1 = contour_area;
				i_max1 = i;
			}
			else if (contour_area > area_max2)
			{
				area_max2 = contour_area;
				i_max2 = i;
			}
		}

		if (i_max1 != -1 && i_max2 != -1)
		{
			if (area_max1 / area_max2 < 1 + DIF_RATIO_2_PART_SIGN_STOP_AREA)
			{
				Rect bound1 = boundingRect(contours[i_max1]);
				Rect bound2 = boundingRect(contours[i_max2]);
				
				rectangle(colorImg, Point(bound1.x, bound1.y), Point(bound1.x + bound1.width, bound1.y + bound1.height), Scalar(0, 255, 255), 2);
				rectangle(colorImg, Point(bound2.x, bound2.y), Point(bound2.x + bound2.width, bound2.y + bound2.height), Scalar(0, 255, 255), 2);

				_sign_ROI.x = min(bound1.x, bound2.x);
				_sign_ROI.y = min(bound1.y, bound2.y);
				_sign_ROI.width = max(bound1.x + bound1.width, bound2.x + bound2.width) - _sign_ROI.x;
				_sign_ROI.height = max(bound1.y + bound1.height, bound2.y + bound2.height) - _sign_ROI.y;
			}
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

void Sign::resetClassID()
{
	_class_id = NO_SIGN;
}

Rect Sign::getROI()
{
	return _sign_ROI;
}
