/*
 * G2oJointOffsetOptimization.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/G2oJointOffsetOptimization.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>

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

#include <kdl/kdl.hpp>

#include <time.h>
#include <map>

using namespace std;

namespace kinematic_calibration {

G2oJointOffsetOptimization::G2oJointOffsetOptimization(
		CalibrationContext& context, vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		JointOffsetOptimization(context, measurements, kinematicChains,
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
	//linearSolver->setTolerance(0.5);
	//linearSolver->setAbsoluteTolerance(true);

	// create the block solver on top of the linear solver
	MyBlockSolver* blockSolver = new MyBlockSolver(linearSolver);
	blockSolver->setLevenberg(true);

	// create the algorithm to carry out the optimization
//	OptimizationAlgorithmGaussNewton* algorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
	OptimizationAlgorithmLevenberg* algorithm =
			new OptimizationAlgorithmLevenberg(blockSolver);

	optimizer.setAlgorithm(algorithm);

	// get the options
	CalibrationOptions options = context.getCalibrationOptions();

	int id = 0;

	// build the graph:

	// instantiate the vertex for the joint offsets
	vector<string> jointNames;
	for (int i = 0; i < kinematicChains.size(); i++) {
		vector<string> currentNames;
		kinematicChains[i].getJointNames(currentNames);
		// add joint names if not already contained
		for (int j = 0; j < currentNames.size() - 1; j++) {
			if (find(jointNames.begin(), jointNames.end(), currentNames[j])
					== jointNames.end()) {
				jointNames.push_back(currentNames[j]);
			}

		}
	}
	JointOffsetVertex* jointOffsetVertex = new JointOffsetVertex(jointNames);
	//jointOffsetVertex->setEstimate(initialState.jointOffsets);
	jointOffsetVertex->setId(++id);
	jointOffsetVertex->setFixed(!options.calibrateJointOffsets);
	//jointOffsetVertex->setFixed(true);
	optimizer.addVertex(jointOffsetVertex);

	// instantiate the vertices for the marker transformations
	map<string, MarkerTransformationVertex*> markerTransformationVertices;
	map<string, KinematicChain> kinematicChainsMap;
	for (int i = 0; i < kinematicChains.size(); i++) {
		MarkerTransformationVertex* markerTransformationVertex =
				new MarkerTransformationVertex();
		markerTransformationVertex->setId(++id);
		markerTransformationVertex->setFixed(!options.calibrateMarkerTransform);
		optimizer.addVertex(markerTransformationVertex);
		KinematicChain currentChain = kinematicChains[i];
		Eigen::Isometry3d markerEigen;
		tfToEigen(initialState.markerTransformations[currentChain.getName()], markerEigen);
		markerTransformationVertex->setEstimate(markerEigen);
		markerTransformationVertices.insert(
				make_pair(currentChain.getName(), markerTransformationVertex));
		kinematicChainsMap.insert(
				make_pair(currentChain.getName(), currentChain));
	}

	// instantiate the vertex for the camera transformation
	TransformationVertex* cameraToHeadTransformationVertex =
			new TransformationVertex();
	cameraToHeadTransformationVertex->setId(++id);
	cameraToHeadTransformationVertex->setFixed(
			!options.calibrateCameraTransform);
	Eigen::Isometry3d initialCameraToHeadIsometry;
	tfToEigen(this->initialState.cameraToHeadTransformation,
			initialCameraToHeadIsometry);
	cameraToHeadTransformationVertex->setEstimate(initialCameraToHeadIsometry);
	optimizer.addVertex(cameraToHeadTransformationVertex);

	// instantiate the vertex for the camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex = new CameraIntrinsicsVertex(
			frameImageConverter.getCameraModel().cameraInfo());
	cameraIntrinsicsVertex->setId(++id);
	cameraIntrinsicsVertex->setFixed(!options.calibrateCameraIntrinsics);
	cameraIntrinsicsVertex->setToOrigin();
	optimizer.addVertex(cameraIntrinsicsVertex);

	// instantiate the vertices for the joint 6D transformations
	map<string, KDL::Frame> framesToTip; // contains all frames (=transformations) to optimize
	for (int i = 0; i < kinematicChains.size(); i++) {
		map<string, KDL::Frame> current = kinematicChains[i].getFramesToTip();
		framesToTip.insert(current.begin(), current.end());
	}
	map<string, g2o::HyperGraph::Vertex*> jointFrameVertices;
	for (map<string, KDL::Frame>::iterator it = framesToTip.begin();
			it != framesToTip.end(); it++) {
		TransformationVertex* vertex = new TransformationVertex();
		Eigen::Affine3d eigenAffine;
		tf::transformKDLToEigen(it->second, eigenAffine);
		Eigen::Isometry3d eigenIsometry;
		eigenIsometry.translation() = eigenAffine.translation();
		eigenIsometry.linear() = eigenAffine.rotation();
		vertex->setEstimate(eigenIsometry);
		vertex->setId(++id);
		vertex->setFixed(true); // TODO: parameterize!
		jointFrameVertices[it->first] = vertex;
		optimizer.addVertex(vertex);
	}

	// add edges
	for (int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];
		if (markerTransformationVertices.count(current.chain_name) == 0
				|| kinematicChainsMap.count(current.chain_name) == 0) {
			ROS_ERROR("Measurement for unknown kinematic chain: %s",
					current.chain_name.c_str());
			continue;
		}
		RobustKernel* rk = new RobustKernelHuber();
		//rk->setDelta(5.0);
		g2o::OptimizableGraph::Edge* edge = context.getMeasurementEdge(current,
				&frameImageConverter,
				&kinematicChainsMap.find(current.chain_name)->second);
		edge->setId(++id);
		//edge->setRobustKernel(rk);
		edge->vertices()[0] = markerTransformationVertices[current.chain_name];
		edge->vertices()[1] = jointOffsetVertex;
		edge->vertices()[2] = cameraToHeadTransformationVertex;
		edge->vertices()[3] = cameraIntrinsicsVertex;
		for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
				jointFrameVertices.begin(); it != jointFrameVertices.end();
				it++) {
			dynamic_cast<EdgeWithJointFrameVertices*>(edge)->setJointFrameVertex(
					it->first, it->second);
		}
		edge->computeError();
		optimizer.addEdge(edge);
	}

	// optimize:
	ROS_INFO("Starting optimization...");
	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	optimizer.setVerbose(true);
	optimizer.optimize(30);

	// get results:

	// joint offsets
	optimizedState.jointOffsets =
			static_cast<map<string, double> >(jointOffsetVertex->estimate());

	// marker transformations
	for (map<string, MarkerTransformationVertex*>::iterator it =
			markerTransformationVertices.begin();
			it != markerTransformationVertices.end(); it++) {
		Eigen::Isometry3d eigenTransform = it->second->estimate();
		tf::Transform tfTransform;
		tf::transformEigenToTF(eigenTransform, tfTransform);
		optimizedState.markerTransformations[it->first] = tfTransform;
	}

	// camera transformation
	Eigen::Isometry3d eigenTransform =
			cameraToHeadTransformationVertex->estimate();
	tf::transformEigenToTF(eigenTransform,
			optimizedState.cameraToHeadTransformation);

	// camera intrinsics
	optimizedState.cameraInfo = cameraIntrinsicsVertex->estimate();

	// joint transformations
	map<string, tf::Transform> jointTransformations;
	for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
			jointFrameVertices.begin(); it != jointFrameVertices.end(); it++) {
		Eigen::Isometry3d eigenTransform =
				static_cast<TransformationVertex*>(it->second)->estimate();
		tf::Transform tfTransform;
		tf::transformEigenToTF(eigenTransform, tfTransform);
		jointTransformations[it->first] = tfTransform;
	}
	optimizedState.jointTransformations = jointTransformations;
}

void G2oJointOffsetOptimization::tfToEigen(const tf::Transform& tfTransformation,
		Eigen::Isometry3d& eigenIsometry) const {
	Eigen::Affine3d eigenAffine;
	tf::transformTFToEigen(tfTransformation, eigenAffine);
	eigenIsometry.translation() = eigenAffine.translation();
	eigenIsometry.linear() = eigenAffine.rotation();
}

} /* namespace kinematic_calibration */

