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
#include <cstdio>

#include "kinematic_calibration/measurementData.h"
#include "../../include/optimization/JointOffsetOptimization.h"
#include "../../include/optimization/KinematicCalibrationState.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/optimization/CheckerboardMeasurementEdge.h"
#include "../../include/optimization/TransformationVertex.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include <kdl/kdl.hpp>

#include <time.h>
#include <map>

using namespace std;

namespace kinematic_calibration {

//typedef TransformationVertex MarkerTransformationVertex;
typedef TransformationVertex CameraTransformationVertex;

class SaveStateHyperGraphAction: public g2o::HyperGraphAction {
public:
	/// Constructor.
	SaveStateHyperGraphAction(vector<KinematicCalibrationState>& states,
			JointOffsetVertex* jointOffsetVertex,
			map<string, TransformationVertex*> markerTransformationVertices,
			TransformationVertex* cameraToHeadTransformationVertex,
			CameraIntrinsicsVertex* cameraIntrinsicsVertex,
			map<string, g2o::HyperGraph::Vertex*> jointFrameVertices);

	/// Desctructor.
	virtual ~SaveStateHyperGraphAction();

	/// Reimplementation of the action.
	virtual HyperGraphAction* operator()(const HyperGraph* graph,
			Parameters* parameters = 0);

	vector<KinematicCalibrationState>& states;
	JointOffsetVertex* jointOffsetVertex;
	map<string, TransformationVertex*> markerTransformationVertices;
	TransformationVertex* cameraToHeadTransformationVertex;
	CameraIntrinsicsVertex* cameraIntrinsicsVertex;
	map<string, g2o::HyperGraph::Vertex*> jointFrameVertices;
};

G2oJointOffsetOptimization::G2oJointOffsetOptimization(
		CalibrationContext& context, vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		JointOffsetOptimization(context, measurements, kinematicChains,
				frameImageConverter, initialState), saveIntermediateStates(
				false) {
}

G2oJointOffsetOptimization::~G2oJointOffsetOptimization() {
}

void G2oJointOffsetOptimization::optimize(
		KinematicCalibrationState& optimizedState) {
	//typedef BlockSolver<BlockSolverTraits<-1, -1> > MyBlockSolver;
	typedef BlockSolverX MyBlockSolver;
	typedef LinearSolverDense<MatrixXd> MyLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;

	// create the linear solver
	MyLinearSolver* linearSolver = new MyLinearSolver();
	//linearSolver->setTolerance(1e-10);
	//linearSolver->setAbsoluteTolerance(true);

	// create the block solver on top of the linear solver
	MyBlockSolver* blockSolver = new MyBlockSolver(linearSolver);
	blockSolver->setLevenberg(true);
	blockSolver->setSchur(false);

	// create the algorithm to carry out the optimization
//	OptimizationAlgorithmGaussNewton* algorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
	OptimizationAlgorithmLevenberg* algorithm =
			new OptimizationAlgorithmLevenberg(blockSolver);
	//algorithm->setMaxTrialsAfterFailure(100);
	algorithm->printVerbose(cout);

	optimizer.setAlgorithm(algorithm);
	//blockSolver->init(&optimizer);

	// get the options
	CalibrationOptions options = context.getCalibrationOptions();
	OptimizationOptions optOptions = context.getOptimizationOptions();

	// compute statistics
	optimizer.setComputeBatchStatistics(true); // TODO: as parameter?

	int id = 0;

	// build the graph:

	// determine the joint optimization type
	JointOptimizationType jointOptType;
	if (options.calibrateJoint6D)
		jointOptType = JOINT_6D;
	else if (options.calibrateJointOffsets)
		jointOptType = JOINT_OFFSETS;
	else
		jointOptType = NONE;

	// instantiate the vertex for the joint offsets
	// TODO: as parameter / from model!
	vector<string> mimicJointsFirst, mimicJointsSecond;
	mimicJointsFirst.push_back("RHipYawPitch");
	mimicJointsSecond.push_back("LHipYawPitch");
	map<string, string> currentMimicJoints;
	vector<string> jointNames;
	int skipLast = options.calibrateMarkerTransform ? 1 : 0;
	int skipFirst = options.calibrateCameraTransform ? 1 : 0;
	for (int i = 0; i < kinematicChains.size(); i++) {
		vector<string> currentNames;
		kinematicChains[i].getJointNames(currentNames);

		for (int j = skipFirst; j < currentNames.size() - skipLast; j++) {
			string currentName = currentNames[j];
			// check if not already contained
			if (find(jointNames.begin(), jointNames.end(), currentNames[j])
					!= jointNames.end()) {
				// joint already contained
				continue;
			}

			// check if current is mimic joint
			for (int k = 0; k < mimicJointsFirst.size(); k++) {
				if (currentName == mimicJointsFirst[k]
						&& find(jointNames.begin(), jointNames.end(),
								mimicJointsSecond[k]) != jointNames.end()) {
					currentMimicJoints[mimicJointsSecond[k]] = currentName;
					continue;
				}
			}
			for (int k = 0; k < mimicJointsSecond.size(); k++) {
				if (currentName == mimicJointsSecond[k]
						&& find(jointNames.begin(), jointNames.end(),
								mimicJointsFirst[k]) != jointNames.end()) {
					currentMimicJoints[mimicJointsFirst[k]] = currentName;
					continue;
				}
			}

			// everything ok -> add the joint
			jointNames.push_back(currentNames[j]);
		}
	}
	JointOffsetVertex* jointOffsetVertex = new JointOffsetVertex(jointNames);
	//jointOffsetVertex->setEstimate(initialState.jointOffsets);
	jointOffsetVertex->setId(++id);
	jointOffsetVertex->setFixed(jointOptType != JOINT_OFFSETS);
	for (map<string, string>::iterator it = currentMimicJoints.begin();
			it != currentMimicJoints.end(); it++) {
		jointOffsetVertex->setMimicJoint(it->first, it->second);
		ROS_INFO("Mimic joints: %s <-> %s.", it->first.c_str(),
				it->second.c_str());
	}
	optimizer.addVertex(jointOffsetVertex);

	// instantiate the vertices for the marker transformations
	map<string, MarkerTransformationVertex*> markerTransformationVertices;
	map<string, KinematicChain> kinematicChainsMap;
	for (int i = 0; i < kinematicChains.size(); i++) {
		MarkerTransformationVertex* markerTransformationVertex;
		if ("single_point" == options.markerOptimizationType) {
			markerTransformationVertex = new TranslationVertex();
		} else {
			// full pose
			markerTransformationVertex = new TransformationVertex();
		}
		markerTransformationVertex->setId(++id);
		markerTransformationVertex->setFixed(!options.calibrateMarkerTransform);
		optimizer.addVertex(markerTransformationVertex);
		KinematicChain currentChain = kinematicChains[i];
		markerTransformationVertex->setEstimateFromTfTransform(
				initialState.markerTransformations[currentChain.getName()]);
		markerTransformationVertices.insert(
				make_pair(currentChain.getName(), markerTransformationVertex));
		kinematicChainsMap.insert(
				make_pair(currentChain.getName(), currentChain));
	}

	// instantiate the vertex for the camera transformation
	CameraTransformationVertex* cameraToHeadTransformationVertex =
			new CameraTransformationVertex();
	cameraToHeadTransformationVertex->setId(++id);
	cameraToHeadTransformationVertex->setEstimateFromTfTransform(
			this->initialState.cameraToHeadTransformation);
	cameraToHeadTransformationVertex->setFixed(
			!options.calibrateCameraTransform);
	optimizer.addVertex(cameraToHeadTransformationVertex);

	// instantiate the vertex for the camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex = new CameraIntrinsicsVertex(
			frameImageConverter.getCameraModel().cameraInfo());
	cameraIntrinsicsVertex->setId(++id);
	cameraIntrinsicsVertex->setToOrigin();
	cameraIntrinsicsVertex->setFixed(!options.calibrateCameraIntrinsics);
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
		vertex->setFixed(jointOptType != JOINT_6D);
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
		if (optOptions.useRobustKernel) {
			edge->setRobustKernel(rk);
		}
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
		dynamic_cast<EdgeWithJointFrameVertices*>(edge)->setJointOptimizationType(
				jointOptType);
		edge->computeError();
		optimizer.addEdge(edge);
	}

	// add action to save the intermediate states
	if (saveIntermediateStates) {
		intermediateStates.push_back(this->initialState);
		optimizer.addPostIterationAction(
				new SaveStateHyperGraphAction(intermediateStates,
						jointOffsetVertex, markerTransformationVertices,
						cameraToHeadTransformationVertex,
						cameraIntrinsicsVertex, jointFrameVertices));
	}

	// early stopping
	if (optOptions.doEarlyStopping) {
		SparseOptimizerTerminateAction* terminateAction =
				new SparseOptimizerTerminateAction();
		terminateAction->setGainThreshold(optOptions.gainThreshold);
		optimizer.addPostIterationAction(terminateAction);
	}

	// optimize:
	ROS_INFO("Starting g2o optimization...");
	optimizer.initializeOptimization();
	//optimizer.computeActiveErrors();
	optimizer.setVerbose(true);
	optimizer.optimize(optOptions.maxIterations);
	ROS_INFO("Done!");

	// get results:

	// joint offsets
	optimizedState.jointOffsets =
			static_cast<map<string, double> >(jointOffsetVertex->estimate());

	// marker transformations
	for (map<string, MarkerTransformationVertex*>::iterator it =
			markerTransformationVertices.begin();
			it != markerTransformationVertices.end(); it++) {
		optimizedState.markerTransformations[it->first] =
				it->second->estimateAsTfTransform();
	}

	// camera transformation
	optimizedState.cameraToHeadTransformation =
			cameraToHeadTransformationVertex->estimateAsTfTransform();

	// camera intrinsics
	optimizedState.cameraInfo = cameraIntrinsicsVertex->estimate();

	// joint transformations
	map<string, tf::Transform> jointTransformations;
	for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
			jointFrameVertices.begin(); it != jointFrameVertices.end(); it++) {
		jointTransformations[it->first] =
				static_cast<TransformationVertex*>(it->second)->estimateAsTfTransform();
	}
	optimizedState.jointTransformations = jointTransformations;

	// plot the error optimization
	ROS_INFO("Writing chi2 plot...");
	this->statistics = optimizer.batchStatistics();
	this->plotStatistics(statistics);
}

BatchStatisticsContainer G2oJointOffsetOptimization::getStatistics() const {
	return this->statistics;
}

void G2oJointOffsetOptimization::setSaveIntermediateStates(
		bool saveIntermediateStates) {
	this->saveIntermediateStates = saveIntermediateStates;
}

void G2oJointOffsetOptimization::getIntermediateStates(
		vector<KinematicCalibrationState>& intermediateStates) const {
	intermediateStates = this->intermediateStates;
}

void G2oJointOffsetOptimization::plotStatistics(
		const BatchStatisticsContainer& statistics) const {
	FILE * gnuplotPipe;
	gnuplotPipe = popen("gnuplot -persistent", "w");
	fprintf(gnuplotPipe,
			"set terminal svg size 350,262 fname 'Verdana' fsize 10\n");
	fprintf(gnuplotPipe, "set output 'chi2.svg'\n");
	fprintf(gnuplotPipe, "set autoscale;\n set xzeroaxis\n set yzeroaxis\n");
	fprintf(gnuplotPipe,
			"plot '-' with linespoints pt 1 lc rgb 'red' title 'chi2' smooth csplines \n");
	;
	for (int i = 0; i < statistics.size(); i++) {
		fprintf(gnuplotPipe, "%i %f \n", statistics[i].iteration,
				statistics[i].chi2);
	}
	fprintf(gnuplotPipe, "e ,\n ");
	fflush(gnuplotPipe);
	fclose(gnuplotPipe);
}

SaveStateHyperGraphAction::SaveStateHyperGraphAction(
		vector<KinematicCalibrationState>& states,
		JointOffsetVertex* jointOffsetVertex,
		map<string, TransformationVertex*> markerTransformationVertices,
		TransformationVertex* cameraToHeadTransformationVertex,
		CameraIntrinsicsVertex* cameraIntrinsicsVertex,
		map<string, g2o::HyperGraph::Vertex*> jointFrameVertices) :
		states(states), jointOffsetVertex(jointOffsetVertex), markerTransformationVertices(
				markerTransformationVertices), cameraToHeadTransformationVertex(
				cameraToHeadTransformationVertex), cameraIntrinsicsVertex(
				cameraIntrinsicsVertex), jointFrameVertices(jointFrameVertices) {
}

SaveStateHyperGraphAction::~SaveStateHyperGraphAction() {
}

HyperGraphAction* SaveStateHyperGraphAction::operator ()(
		const HyperGraph* graph, Parameters* parameters) {
	KinematicCalibrationState state;

	// joint offsets
	state.jointOffsets =
			static_cast<map<string, double> >(jointOffsetVertex->estimate());

	// marker transformations
	for (map<string, MarkerTransformationVertex*>::iterator it =
			markerTransformationVertices.begin();
			it != markerTransformationVertices.end(); it++) {
		state.markerTransformations[it->first] =
				it->second->estimateAsTfTransform();
	}

	// camera transformation
	state.cameraToHeadTransformation =
			cameraToHeadTransformationVertex->estimateAsTfTransform();

	// camera intrinsics
	state.cameraInfo = cameraIntrinsicsVertex->estimate();

	// joint transformations
	map<string, tf::Transform> jointTransformations;
	for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
			jointFrameVertices.begin(); it != jointFrameVertices.end(); it++) {
		jointTransformations[it->first] =
				static_cast<TransformationVertex*>(it->second)->estimateAsTfTransform();
	}
	state.jointTransformations = jointTransformations;

	states.push_back(state);

	return this;
}

} /* namespace kinematic_calibration */

