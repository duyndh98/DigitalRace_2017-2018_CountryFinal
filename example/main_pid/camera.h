#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <opencv2/opencv.hpp>

class camera {
public:
	virtual ~camera() = default;

	virtual char read_frame( cv::Mat& color_img, cv::Mat& depth_img ) const = 0;
};

#endif //!__CAMERA_H__
