#ifndef __REAL_CAMERA_H__
#define __REAL_CAMERA_H__

#include "camera.h"
#include "header.h"	

class real_camera : public camera {
private:
	Device device;
	VideoStream *depth, *color;

public:
	real_camera();

	virtual ~real_camera();

	char read_frame( cv::Mat& color_img, cv::Mat& depth_img ) const override;
};

#endif //!__REAL_CAMERA_H__
