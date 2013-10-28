/*
 * FrameImageConverter.cpp
 *
 *  Created on: 28.10.2013
 *      Author: stefan
 */

#include "../include/FrameImageConverter.h"

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
    cv::Point2d uv;
    uv = cameraModel.project3dToPixel(pt_cv);
    x = uv.x;
    y = uv.y;
}

} /* namespace kinematic_calibration */

