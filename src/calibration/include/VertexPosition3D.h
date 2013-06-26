/*
 * VertexPosition3D.h
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#ifndef VERTEXPOSITION3D_H_
#define VERTEXPOSITION3D_H_

#include <g2o/core/base_vertex.h>

using namespace g2o;

/*
 * Vertex that represents a 3D point.
 */
class VertexPosition3D : public BaseVertex<3, Eigen::Vector3d> {
public:
	VertexPosition3D();
	virtual ~VertexPosition3D();

	virtual bool read(std::istream&);
	virtual bool write(std::ostream&) const;
	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();

};

#endif /* VERTEXPOSITION3D_H_ */
