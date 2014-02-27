/*
 * TransformationVertex.h
 *
 *  Created on: 27.02.2014
 *      Author: stefan
 */

#ifndef TRANSFORMATIONVERTEX_H_
#define TRANSFORMATIONVERTEX_H_

#include <tf/tf.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace kinematic_calibration {

using namespace g2o;

/**
 * Vertex for 6D transformation.
 */
class TransformationVertex : public VertexSE3 {
public:
	/**
	 * Constructor.
	 */
	TransformationVertex();

	/**
	 * Desctructor.
	 */
	virtual ~TransformationVertex();

	/**
	 * Return the current estimate as tf::Transform.
	 * @return the current estimate as tf::Transform.
	 */
	tf::Transform estimateAsTfTransform() const;

	/**
	 * Set the estimate from tf::Transform.
	 * @param[in] tfTransform the estimate to be set.
	 */
	void setEstimateFromTfTransform(const tf::Transform& tfTransform);

protected:
	/**
	 * Converts from tf::Transform to Eigen::Isometry3d.
	 * @param[in] tfTransformation Transform to be converted.
	 * @param[out] eigenIsometry Result.
	 */
	void tfToEigen(const tf::Transform& tfTransformation,
			Eigen::Isometry3d& eigenIsometry) const;
};

} /* namespace kinematic_calibration */

#endif /* TRANSFORMATIONVERTEX_H_ */
