/*
 * FrameImageConverter.cpp
 *
 *  Created on: 28.10.2013
 *      Author: stefan
 */

#include "../../include/common/FrameImageConverter.h"

#include <opencv2/core/core.hpp>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

namespace kinematic_calibration {

FrameImageConverter::FrameImageConverter(
		image_geometry::PinholeCameraModel cameraModel) :
		cameraModel(cameraModel) {
}

FrameImageConverter::~FrameImageConverter() {

}

void FrameImageConverter::project(const tf::Transform& transform, double& x,
		double& y) {
	tf::Point pt = transform.getOrigin();
	cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
	cv::Point2d uv, uv_rect;
	uv_rect = cameraModel.project3dToPixel(pt_cv);
	uv = cameraModel.unrectifyPoint(uv_rect);
	x = uv.x;
	y = uv.y;
	//std::cout << "rectified: " << uv_rect.x << ", " << uv_rect.y
	//		<< "\tunrectified: " << x << ", " << y << "\n";
}

} /* namespace kinematic_calibration */

