/*
 * VertexTransformation3D.h
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#ifndef VERTEXTRANSFORMATION3D_H_
#define VERTEXTRANSFORMATION3D_H_

// g2o specific includes
#include <g2o/core/base_vertex.h>

// tf specific includes
#include <tf/tf.h>

using namespace g2o;

/*
 * Vertex that represents a 3D transformation.
 */
class VertexTransformation3D :  public BaseVertex<6, tf::Transform> {
public:
	VertexTransformation3D();
	virtual ~VertexTransformation3D();

	virtual bool read(std::istream& in);
	virtual bool write(std::ostream& out) const;
	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();

};

#endif /* VERTEXTRANSFORMATION3D_H_ */
