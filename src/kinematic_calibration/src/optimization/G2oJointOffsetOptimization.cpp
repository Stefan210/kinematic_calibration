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

CheckerboardMeasurementEdge::CheckerboardMeasurementEdge(
		measurementData measurement) :
		measurement(measurement) {
	resize(4);
	for (int i = 0; i < measurement.jointState.name.size(); i++) {
		jointPositions.insert(
				make_pair<string, double>(measurement.jointState.name[i],
						measurement.jointState.position[i]));
	}
}

CheckerboardMeasurementEdge::~CheckerboardMeasurementEdge() {
}

void CheckerboardMeasurementEdge::computeError() {
	// get the pointers to the vertices
	MarkerTransformationVertex* markerTransformationVertex =
			static_cast<MarkerTransformationVertex*>(this->_vertices[0]);
	JointOffsetVertex* jointOffsetVertex =
			static_cast<JointOffsetVertex*>(this->_vertices[1]);
	TransformationVertex* cameraToHeadTransformationVertex =
			static_cast<TransformationVertex*>(this->_vertices[2]);
	CameraIntrinsicsVertex* cameraIntrinsicsVertex =
			static_cast<CameraIntrinsicsVertex*>(this->_vertices[3]);

	// get transformation from end effector to camera
	tf::Transform cameraToEndEffector; // root = camera, tip = end effector, e.g. wrist
	map<string, double> jointOffsets = jointOffsetVertex->estimate();
	this->kinematicChain->getRootToTip(jointPositions, jointOffsets,
			cameraToEndEffector);

	// get transformation from marker to end effector
	Eigen::Isometry3d eigenTransform = markerTransformationVertex->estimate();
	tf::Transform endEffectorToMarker;
	tf::transformEigenToTF(eigenTransform, endEffectorToMarker);

	// get transformation from camera to head
	eigenTransform = cameraToHeadTransformationVertex->estimate();
	tf::Transform cameraToHead;
	tf::transformEigenToTF(eigenTransform, cameraToHead);

	// get estimated camera intrinsics
	sensor_msgs::CameraInfo cameraInfo = this->frameImageConverter->getCameraModel().cameraInfo();
	cameraInfo.K = cameraIntrinsicsVertex->estimate();
	this->frameImageConverter->getCameraModel().fromCameraInfo(cameraInfo);

	// calculate estimated x and y
	endEffectorToMarker.setRotation(tf::Quaternion::getIdentity());
	tf::Transform cameraToMarker = endEffectorToMarker * cameraToEndEffector
			* cameraToHead;
	double x, y;
	this->frameImageConverter->project(cameraToMarker.inverse(), x, y);

	// set error
	this->_error[0] = measurement.cb_x - x;
	this->_error[1] = measurement.cb_y - y;

	/*cout << "id: " << _id << " ";
	 cout << "cameraToEndEffector(x,y,z): " << cameraToEndEffector.getOrigin().getX() << ", "
	 << cameraToEndEffector.getOrigin().getY() << ", " << cameraToEndEffector.getOrigin().getZ()
	 << " ";
	 cout << "x error: " << this->_error[0] << " y error: " << this->_error[1]
	 << " ";
	 cout << "cb_x: " << measurement.cb_x << " cb_y: " << measurement.cb_y;
	 cout << "\n";*/
}

const FrameImageConverter* CheckerboardMeasurementEdge::getFrameImageConverter() const {
	return frameImageConverter;
}

void CheckerboardMeasurementEdge::setFrameImageConverter(
		FrameImageConverter* frameImageConverter) {
	this->frameImageConverter = frameImageConverter;
}

const KinematicChain* CheckerboardMeasurementEdge::getKinematicChain() const {
	return kinematicChain;
}

void CheckerboardMeasurementEdge::setKinematicChain(
		KinematicChain* kinematicChain) {
	this->kinematicChain = kinematicChain;
}

} /* namespace kinematic_calibration */

