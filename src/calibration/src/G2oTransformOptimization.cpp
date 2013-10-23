/*
 * G2oTransformOptimization.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/G2oTransformOptimization.h"
#include "../include/EdgeMarkerMeasurement.h"
#include "../include/EdgeGroundMeasurement.h"
#include "../include/VertexPosition3D.h"
#include "../include/VertexTransformation3D.h"
#include "../include/VertexOffset.h"
#include "../include/MarkerEstimation.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

G2oTransformOptimization::G2oTransformOptimization(
		CameraTransformOptimizationParameter param) :
		CameraTransformOptimization(param) {
	this->markerPositionOptimized = false;
	this->correlationMatrix = Eigen::Matrix<double, 5, 5>::Identity();
	this->correlationMatrix.setConstant(0, 0, parameter.getMarkerWeight());
	this->correlationMatrix.setConstant(1, 1, parameter.getMarkerWeight());
	this->correlationMatrix.setConstant(2, 2, parameter.getMarkerWeight());
	this->correlationMatrix.setConstant(3, 3, parameter.getGroundWeight());
	this->correlationMatrix.setConstant(4, 4, parameter.getGroundWeight());
}

G2oTransformOptimization::~G2oTransformOptimization() {
	// TODO Auto-generated destructor stub
}

void G2oTransformOptimization::optimizeTransform(
		CalibrationState& calibrationState) {

	typedef BlockSolver<BlockSolverTraits<-1, -1> > MyBlockSolver;
	typedef LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;
	//typedef LinearSolverPCG<MyBlockSolver::PoseMatrixType> MyLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;

	// create the linear solver
	MyLinearSolver* linearSolver = new MyLinearSolver();
	//linearSolver->setTolerance(1e-9);
	//linearSolver->setVerbose(true);

	// create the block solver on top of the linear solver
	MyBlockSolver* blockSolver = new MyBlockSolver(linearSolver);
	blockSolver->setLevenberg(true);

	// create the algorithm to carry out the optimization
//	OptimizationAlgorithmGaussNewton* algorithm = new OptimizationAlgorithmGaussNewton(blockSolver);

	OptimizationAlgorithmLevenberg* algorithm =
			new OptimizationAlgorithmLevenberg(blockSolver);
	algorithm->setMaxTrialsAfterFailure(10000);
//	algorithm->setUserLambdaInit(1/1000);

	optimizer.setAlgorithm(algorithm);

	// object for estimating the marker position depending on the current estimations
	MarkerEstimation markerEstimation(this->measurePoints);

	// add a vertex representing the estimated marker position
	/*
	 VertexPosition3D* positionVertex = new VertexPosition3D();
	 tf::Vector3 position;
	 getMarkerEstimate(CalibrationState(initialTransformCameraToHead, 0, 0),
	 position);
	 positionVertex->setEstimate(
	 Eigen::Vector3d(position[0], position[1], position[2]));
	 positionVertex->setId(1);
	 optimizer.addVertex(positionVertex);
	 */

	// add a vertex representing the current transformation
	VertexTransformation3D* transformationVertex = new VertexTransformation3D();
	transformationVertex->setEstimate(getInitialCameraToHead());
	transformationVertex->setId(2);
	optimizer.addVertex(transformationVertex);

	// add a vertex representing the offset for headYaw and headPitch joint
	VertexOffset* offsetVertex = new VertexOffset();
	offsetVertex->setEstimate(Eigen::Matrix<double, 2, 1>(0.0, 0.0));
	offsetVertex->setId(3);
	offsetVertex->setFixed(!parameter.isCalibrateJointOffsets());
	optimizer.addVertex(offsetVertex);

	int id = 1;

	// add edges constraining marker position and transformation
	Eigen::Matrix3d mm = Eigen::Matrix3d::Identity(3, 3);
	mm(0, 0) = parameter.getMarkerWeight();
	mm(1, 1) = parameter.getMarkerWeight();
	mm(2, 2) = parameter.getMarkerWeight();
	for (int i = 0; i < this->measurePoints.size(); i++) {
		EdgeMarkerMeasurement* edge = new EdgeMarkerMeasurement(
				this->measurePoints[i], markerEstimation);
		edge->setId(id++);
		edge->setInformation(mm);
		/*
		 edge->vertices()[0] = positionVertex;
		 edge->vertices()[1] = transformationVertex;
		 edge->vertices()[2] = offsetVertex;
		 */
		edge->vertices()[0] = transformationVertex;
		edge->vertices()[1] = offsetVertex;
		optimizer.addEdge(edge);
	}

	// add edges constraining roll and pitch of the ground
	Eigen::Matrix2d mg = Eigen::Matrix2d::Identity(2, 2);
	mg(0, 0) = parameter.getGroundWeight();
	mg(1, 1) = parameter.getGroundWeight();
	for (int i = 0; i < this->measurePoints.size(); i++) {
		EdgeGroundMeasurement* edge = new EdgeGroundMeasurement(
				this->measurePoints[i], parameter.getGroundDistance());
		edge->setId(id++);
		edge->setInformation(mg);
		edge->vertices()[0] = transformationVertex;
		edge->vertices()[1] = offsetVertex;
		optimizer.addEdge(edge);
	}

	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	//optimizer.setVerbose(true);
	optimizer.optimize(1000);

	double headYawOffset = offsetVertex->estimate()[0];
	double headPitchOffset = offsetVertex->estimate()[1];

	calibrationState.setCameraToHead(transformationVertex->estimate());
	calibrationState.setHeadYawOffset(
			static_cast<double>(offsetVertex->estimate()[0]));
	calibrationState.setHeadPitchOffset(
			static_cast<double>(offsetVertex->estimate()[1]));

	/*
	 this->markerPosition = tf::Vector3(positionVertex->estimate()[0],
	 positionVertex->estimate()[1], positionVertex->estimate()[2]);
	 */

	Eigen::Vector3d markerPositionEigen;
	markerPositionEigen = markerEstimation.estimateMarkerPosition(
			calibrationState);
	this->markerPosition[0] = markerPositionEigen[0];
	this->markerPosition[1] = markerPositionEigen[1];
	this->markerPosition[2] = markerPositionEigen[2];
	this->markerPositionOptimized = true;

	/*std::cout << "offset(yaw,pitch) " << headYawOffset << " " << headPitchOffset
			<< ";";

	std::cout << "position (x,y,z):" << markerPosition[0] << ","
			<< markerPosition[1] << "," << markerPosition[2] << ";";
	std::cout << "translation (x,y,z):"
			<< transformationVertex->estimate().getOrigin()[0] << ","
			<< transformationVertex->estimate().getOrigin()[1] << ","
			<< transformationVertex->estimate().getOrigin()[2] << ";";
	std::cout << "rotation (q0,q1,q2,q3):"
			<< transformationVertex->estimate().getRotation()[0] << ","
			<< transformationVertex->estimate().getRotation()[1] << ","
			<< transformationVertex->estimate().getRotation()[2] << ","
			<< transformationVertex->estimate().getRotation()[3] << ";";*/
}

void G2oTransformOptimization::getMarkerEstimate(const CalibrationState& state,
		tf::Vector3& position) {
	if (this->markerPositionOptimized == true) {
		position = this->markerPosition;
	} else {
		this->CameraTransformOptimization::getMarkerEstimate(state, position);
	}
}

