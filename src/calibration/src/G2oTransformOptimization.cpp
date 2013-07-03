/*
 * G2oTransformOptimization.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/G2oTransformOptimization.h"
#include "../include/EdgeMarkerMeasurement.h"
#include "../include/VertexPosition3D.h"
#include "../include/VertexTransformation3D.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>

G2oTransformOptimization::G2oTransformOptimization() {
	this->markerPositionOptimized = false;
}

G2oTransformOptimization::~G2oTransformOptimization() {
	// TODO Auto-generated destructor stub
}

void G2oTransformOptimization::optimizeTransform(tf::Transform& cameraToHead) {

	typedef BlockSolver<BlockSolverTraits<-1, -1> > MyBlockSolver;
	typedef LinearSolverPCG<MyBlockSolver::PoseMatrixType> MyLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;

	// create the linear solver
	MyLinearSolver* linearSolver = new MyLinearSolver();
	linearSolver->setTolerance(1e-9);
	//linearSolver->setVerbose(true);

	// create the block solver on top of the linear solver
	MyBlockSolver* blockSolver = new MyBlockSolver(linearSolver);

	// create the algorithm to carry out the optimization
	//OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
	OptimizationAlgorithmLevenberg* algorithm =
			new OptimizationAlgorithmLevenberg(blockSolver);

	optimizer.setAlgorithm(algorithm);

	VertexPosition3D* positionVertex = new VertexPosition3D();
	tf::Vector3 position;
	getMarkerEstimate(initialTransformCameraToHead, position);
	positionVertex->setEstimate(
			Eigen::Vector3d(position[0], position[1], position[2]));
	positionVertex->setId(1);
	optimizer.addVertex(positionVertex);

	VertexTransformation3D* transformationVertex = new VertexTransformation3D();
	transformationVertex->setEstimate(initialTransformCameraToHead);
	transformationVertex->setId(2);
	optimizer.addVertex(transformationVertex);

	for (int i = 0; i < this->measurePoints.size(); i++) {
		EdgeMarkerMeasurement* edge = new EdgeMarkerMeasurement(
				this->measurePoints[i]);
		edge->setId(i);
		edge->setInformation(Eigen::Matrix<double, 5, 5>::Identity());
		edge->vertices()[0] = positionVertex;
		edge->vertices()[1] = transformationVertex;
		optimizer.addEdge(edge);
	}

	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	optimizer.setVerbose(true);
	optimizer.optimize(15000);
	cameraToHead = transformationVertex->estimate();

	this->markerPosition = tf::Vector3(positionVertex->estimate()[0],
			positionVertex->estimate()[1], positionVertex->estimate()[2]);
	this->markerPositionOptimized = true;

	/*std::cout << "position (x,y,z):" << positionVertex->estimate()[0] << ","
	 << positionVertex->estimate()[1] << ","
	 << positionVertex->estimate()[2] << ";";
	 std::cout << "translation (x,y,z):" << transformationVertex->estimate().getOrigin()[0]
	 << "," << transformationVertex->estimate().getOrigin()[1] << ","
	 << transformationVertex->estimate().getOrigin()[2] << ";";
	 std::cout << "rotation (q0,q1,q2,q3):"
	 << transformationVertex->estimate().getRotation()[0] << ","
	 << transformationVertex->estimate().getRotation()[1] << ","
	 << transformationVertex->estimate().getRotation()[2] << ","
	 << transformationVertex->estimate().getRotation()[3] << ";";*/
}

void G2oTransformOptimization::getMarkerEstimate(
		const tf::Transform& cameraToHead, tf::Vector3& position) {
	if (this->markerPositionOptimized == true) {
		position = this->markerPosition;
	} else {
		this->CameraTransformOptimization::getMarkerEstimate(cameraToHead, position);
	}
}

