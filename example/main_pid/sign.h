#ifndef __SIGN_H__
#define __SIGN_H__

#include "header.h"
#include "image_processing.h"

class Sign
{
private:
	HOGDescriptor _hog;
	Ptr<SVM> _svm;
	Rect _sign_ROI;
	int _class_id;

public:
	Sign();
	bool detect(bool blueSign);
	void recognize();
	void classify(Mat &graySignimg);
	int getClassID();
	void resetClassID();
	Rect getROI();
};

#endif
