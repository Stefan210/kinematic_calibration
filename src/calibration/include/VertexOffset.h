/*
 * VertexOffset.h
 *
 *  Created on: 25.07.2013
 *      Author: stefan
 */

#ifndef VERTEXOFFSET_H_
#define VERTEXOFFSET_H_

#include <g2o/core/base_vertex.h>

using namespace g2o;

/*
 * Represents a node for headYaw and headPitch joint offset.
 */
class VertexOffset : public BaseVertex<2, Eigen::Vector2d>  {
public:
	VertexOffset();
	virtual ~VertexOffset();

	virtual bool read(std::istream&);
	virtual bool write(std::ostream&) const;
	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();
};

#endif /* VERTEXOFFSET_H_ */
