#include "real_camera.h"
#include "my_assert.h"

real_camera::real_camera() {
	// init Open NI
	MY_ASSERT( OpenNI::initialize() == STATUS_OK );
	MY_ASSERT( device.open( ANY_DEVICE ) == STATUS_OK );

	// start depth stream
	depth = new VideoStream();
	if ( device.getSensorInfo( SENSOR_DEPTH ) != NULL ) {
		MY_ASSERT( depth->create( device, SENSOR_DEPTH ) == STATUS_OK );
		VideoMode depth_mode = depth->getVideoMode();
		depth_mode.setFps( 30 );
		depth_mode.setResolution( FRAME_WIDTH, FRAME_HEIGHT );
		depth_mode.setPixelFormat( PIXEL_FORMAT_DEPTH_100_UM );
		depth->setVideoMode( depth_mode );
		MY_ASSERT( depth->start() == STATUS_OK );
	}

	// start color stream
	color = new VideoStream();
	if ( device.getSensorInfo( SENSOR_COLOR ) != NULL ) {
		MY_ASSERT( color->create( device, SENSOR_COLOR ) == STATUS_OK );
		VideoMode color_mode = color->getVideoMode();
		color_mode.setFps( 30 );
		color_mode.setResolution( FRAME_WIDTH, FRAME_HEIGHT );
		color_mode.setPixelFormat( PIXEL_FORMAT_RGB888 );
		color->setVideoMode( color_mode );
		MY_ASSERT( color->start() == STATUS_OK );
	}
}

real_camera::~real_camera() {
	delete depth;
	delete color;
	device.close();
}

char real_camera::read_frame( cv::Mat& color_img, cv::Mat& depth_img ) const {
	VideoFrameRef frame_depth, frame_color;
	VideoStream *streams[] = {depth, color};
	
	int readyStream = -1;

	MY_ASSERT( OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT) == STATUS_OK );
	depth->readFrame(&frame_depth);
	color->readFrame(&frame_color);

	DepthPixel *depth_img_data;
	RGB888Pixel *color_img_data;

	int w = frame_color.getWidth();
	int h = frame_color.getHeight();

	depth_img = Mat(h, w, CV_16U);
	color_img = Mat(h, w, CV_8UC3);

	// read depth
	Mat depth_img_8u; // for debug
	depth_img_data = (DepthPixel *)frame_depth.getData();
	memcpy(depth_img.data, depth_img_data, h * w * sizeof(DepthPixel));
	normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
	depth_img_8u.convertTo(depth_img_8u, CV_8U);

	// read color
	color_img_data = (RGB888Pixel *)frame_color.getData();
	memcpy(color_img.data, color_img_data, h * w * sizeof(RGB888Pixel));
	cvtColor(color_img, color_img, COLOR_RGB2BGR);

	return 'c';
}
