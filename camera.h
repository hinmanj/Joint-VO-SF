
#include <Eigen/Core>
#include <OpenNI.h>
#include <iostream>

#include <librealsense\rs.hpp>


class RGBD_Camera {
public:

    RGBD_Camera(unsigned int res_factor);

	unsigned int cam_mode;
	float max_distance;

    openni::Status		rc;
    openni::Device		device;
    openni::VideoMode	options;
    openni::VideoStream rgb,dimage;

    bool openCamera();
    void closeCamera();
    void loadFrame(Eigen::MatrixXf &depth_wf, Eigen::MatrixXf &color_wf);
    void disableAutoExposureAndWhiteBalance();
};

class RealSense_Camera {
public:

	RealSense_Camera(unsigned int res_factor);

	unsigned int cam_mode;
	float max_distance;


	rs::intrinsics depth_intrin;
	rs::intrinsics color_intrin;
	rs::context ctx;

	//openni::Status		rc;
	rs::device*		device;
	//openni::VideoMode	options;
	uint16_t* dimage;
	uint8_t* rgb;

	bool openCamera();
	void closeCamera();
	void loadFrame(Eigen::MatrixXf &depth_wf, Eigen::MatrixXf &color_wf);
	void disableAutoExposureAndWhiteBalance();
};


