/*
 * CameraIntrinsicsVertex.h
 *
 *  Created on: 06.12.2013
 *      Author: stefan
 */

#ifndef CAMERAINTRINSICSVERTEX_H_
#define CAMERAINTRINSICSVERTEX_H_

#include <boost/array.hpp>
#include <g2o/core/base_vertex.h>
#include <sensor_msgs/CameraInfo.h>

using namespace g2o;

namespace kinematic_calibration {

typedef boost::array<double, 9> CameraIntrinsicsType;

/*
 *
 */
class CameraIntrinsicsVertex: BaseVertex<4, CameraIntrinsicsType> {
public:
	CameraIntrinsicsVertex(sensor_msgs::CameraInfo cameraInfo);
	virtual ~CameraIntrinsicsVertex();

	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}
	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();

protected:
	sensor_msgs::CameraInfo cameraInfo;

};

} /* namespace kinematic_calibration */

#endif /* CAMERAINTRINSICSVERTEX_H_ */
