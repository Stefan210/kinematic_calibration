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
class TransformationVertex: public VertexSE3 {
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

	void setFixed(bool fixed) {
		VertexSE3::setFixed(fixed);
	}
	void setId(int newId) {
		VertexSE3::setId(newId);
	}

protected:
	/**
	 * Converts from tf::Transform to Eigen::Isometry3d.
	 * @param[in] tfTransformation Transform to be converted.
	 * @param[out] eigenIsometry Result.
	 */
	void tfToEigen(const tf::Transform& tfTransformation,
			Eigen::Isometry3d& eigenIsometry) const;
};

class TranslationVertex: public TransformationVertex {
public:
	TranslationVertex() {
	}
	virtual ~TranslationVertex() {
	}
	;

	void oplusImpl(const double* delta) {
		std::vector<double> deltaVec;
		for (int i = 0; i < 3; i++)
			deltaVec.push_back(delta[i]);

		for (int i = 0; i < 3; i++)
			deltaVec.push_back(0.0);

		TransformationVertex::oplusImpl(&deltaVec[0]);
	}

	int estimateDimension() const {
		return 3;
	}

	int minimalEstimateDimension() const {
		return 3;
	}

protected:
	tf::Vector3 translation;
};

typedef TransformationVertex MarkerTransformationVertex;

} /* namespace kinematic_calibration */

#endif /* TRANSFORMATIONVERTEX_H_ */
