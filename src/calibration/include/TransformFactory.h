/*
 * TransformFactory.h
 *
 *  Created on: 04.06.2013
 *      Author: stefan
 */

#ifndef TRANSFORMFACTORY_H_
#define TRANSFORMFACTORY_H_

// TF specific includes
#include <tf/tf.h>
#include <tf/transform_listener.h>

/*
 *
 */
class TransformFactory {
public:
	TransformFactory();
	virtual ~TransformFactory();
	virtual void getTransform(tf::Transform& transform) = 0;
};

class TfTransformFactory : public TransformFactory {
public:
	TfTransformFactory(std::string targetFrame, std::string sourceFrame);
	virtual ~TfTransformFactory();
	virtual void getTransform(tf::Transform& transform);

protected:
	std::string targetFrame;
	std::string sourceFrame;
	tf::TransformListener transformListener;
};

class ManualTransformFactory : public TransformFactory {
public:
	ManualTransformFactory(float tx, float ty, float tz, float roll, float pitch, float yaw);
	virtual ~ManualTransformFactory();
	virtual void getTransform(tf::Transform& transform);

protected:
	float tx;
	float ty;
	float tz;
	float roll;
	float pitch;
	float yaw;
};

#endif /* TRANSFORMFACTORY_H_ */
