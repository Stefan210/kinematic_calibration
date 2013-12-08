/*
 * G2oJointOffsetOptimization.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/G2oJointOffsetOptimization.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "kinematic_calibration/measurementData.h"
#include "../../include/optimization/JointOffsetOptimization.h"
#include "../../include/optimization/KinematicCalibrationState.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/optimization/CheckerboardMeasurementEdge.h"

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
#include <g2o/core/robust_kernel_impl.h>

#include <time.h>

namespace kinematic_calibration {

G2oJointOffsetOptimization::G2oJointOffsetOptimization(
		vector<measurementData>& measurements, KinematicChain& kinematicChain,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		JointOffsetOptimization(measurements, kinematicChain,
				frameImageConverter, initialState) {
}

G2oJointOffsetOptimization::~G2oJointOffsetOptimization() {
}

void G2oJointOffsetOptimization::optimize(
		KinematicCalibrationState& optimizedState) {
	typedef BlockSolver<BlockSolverTraits<-1, -1> > MyBlockSolver;
	//typedef LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;
	typedef LinearSolverPCG<MyBlockSolver::PoseMatrixType> MyLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;

	// create the linear solver
	MyLinearSolver* linearSolver = new MyLinearSolver();

	// create the block solver on top of the linear solver
	MyBlockSolver* blockSolver = new MyBlockSolver(linearSolver);
	blockSolver->setLevenberg(true);

	// create the algorithm to carry out the optimization
//	OptimizationAlgorithmGaussNewton* algorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
	OptimizationAlgorithmLevenberg* algorithm =
			new OptimizationAlgorithmLevenberg(blockSolver);

	optimizer.setAlgorithm(algorithm);

	int id = 0;

	// instantiate the vertex for the joint offsets
	vector<string> jointNames;
	kinematicChain.getJointNames(jointNames);
	JointOffsetVertex* jointOffsetVertex = new JointOffsetVertex(jointNames);
	jointOffsetVertex->setId(++id);
	optimizer.addVertex(jointOffsetVertex);

	// instantiate the vertex for the marker transformation
	MarkerTransformationVertex* markerTransformationVertex =
			new MarkerTransformationVertex();
	markerTransformationVertex->setId(++id);
	optimizer.addVertex(markerTransformationVertex);

	// instantiate the vertex for the marker transformation
	TransformationVertex* cameraToHeadTransformationVertex =
			new TransformationVertex();
	cameraToHeadTransformationVertex->setId(++id);
	optimizer.addVertex(cameraToHeadTransformationVertex);

	// instantiate the vertex for the camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex = new CameraIntrinsicsVertex(
			frameImageConverter.getCameraModel().cameraInfo());
	cameraIntrinsicsVertex->setId(++id);
	cameraIntrinsicsVertex->setToOrigin();
	//cameraIntrinsicsVertex->setFixed(true);
	optimizer.addVertex(cameraIntrinsicsVertex);

	// add edges
	Eigen::Matrix3d info = Eigen::Matrix3d::Identity(3, 3);
	for (int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];
		CheckerboardMeasurementEdge* edge = new CheckerboardMeasurementEdge(
				current);
		edge->setId(++id);
		edge->setInformation(info);
		edge->vertices()[0] = markerTransformationVertex;
		edge->vertices()[1] = jointOffsetVertex;
		edge->vertices()[2] = cameraToHeadTransformationVertex;
		edge->vertices()[3] = cameraIntrinsicsVertex;
		edge->setFrameImageConverter(&frameImageConverter);
		edge->setKinematicChain(&kinematicChain);
		edge->computeError();
		optimizer.addEdge(edge);
	}

	// optimize
	ROS_INFO("Starting optimization...");
	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	optimizer.setVerbose(true);
	optimizer.optimize(100000);

	// get results
	optimizedState.jointOffsets =
			static_cast<map<string, double> >(jointOffsetVertex->estimate());
	Eigen::Isometry3d eigenTransform = markerTransformationVertex->estimate();
	tf::transformEigenToTF(eigenTransform, optimizedState.markerTransformation);
	eigenTransform = cameraToHeadTransformationVertex->estimate();
	tf::transformEigenToTF(eigenTransform,
			optimizedState.cameraToHeadTransformation);
	optimizedState.cameraK = cameraIntrinsicsVertex->estimate();
}

} /* namespace kinematic_calibration */

