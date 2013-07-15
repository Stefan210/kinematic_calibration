/*
 * GroundDetection.cpp
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#include "../include/GroundDetection.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <math.h>

#include <Eigen/Dense>

GroundDetection::GroundDetection() {
	// TODO Auto-generated constructor stub

}

GroundDetection::~GroundDetection() {
	// TODO Auto-generated destructor stub
}

GroundData GroundDetection::getGroundData(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud) {
	GroundData gd;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDistanceFiltered =
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
					new pcl::PointCloud<pcl::PointXYZRGB>());

	filterRange(initialCloud, cloudDistanceFiltered);
	//segmentPlane(cloudDistanceFiltered, gd);
	segmentPlane(initialCloud, gd);

	if (isnan(gd.a) || isnan(gd.b) || isnan(gd.c))
		throw "Could not detect the plane: One or more of the plane coefficients is nan.";

	return gd;
}

void GroundDetection::filterRange(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredX = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredXY = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	pass.setInputCloud(inCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-1.5, 1.5); //-1.0, 1.0
	pass.filter(*filteredX);

	pass.setInputCloud(filteredX);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.5, 1.5); //-1.0, 1.0
	pass.filter(*filteredXY);

	pass.setInputCloud(filteredXY);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.5, 1.5); //-1.0, 1.0
	pass.filter(*outCloud);
}

void GroundDetection::segmentPlane(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, GroundData& gd) {
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segPlane;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(inCloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	segPlane.setOptimizeCoefficients(true);
	segPlane.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	segPlane.setNormalDistanceWeight(0.1);
	segPlane.setMethodType(pcl::SAC_RANSAC);
	segPlane.setMaxIterations(100);
	segPlane.setDistanceThreshold(0.001);
	segPlane.setInputCloud(inCloud);
	segPlane.setInputNormals(cloud_normals);

	// Obtain the plane inliers and coefficients
	segPlane.segment(*inliers_plane, *coefficients_plane);

	gd.setEquation(coefficients_plane->values[0], coefficients_plane->values[1],
			coefficients_plane->values[2], coefficients_plane->values[3]);
}

tf::Pose GroundData::getPose() const {
	double roll, pitch, yaw;
	getRPY(roll, pitch, yaw);
	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	tf::Pose pose(q, tf::Vector3(0, 0, 0));
	return pose;
}

void GroundData::getRPY(double& roll, double& pitch, double& yaw) const {
	// normal vector to the XY-plane
	tf::Vector3 normalXY(0, 0, 1);

	// roll is the angle between the normal vector projected into the YZ-plane
	roll = normalize(normalXY.angle(tf::Vector3(0, b, c)));

	// pitch is the angle between the normal vector projected into the XZ-plane
	pitch = normalize(normalXY.angle(tf::Vector3(a, 0, c)));
	//std::cout << "(roll, pitch)(1): " << roll << " " << pitch << std::endl;

	// NOTE: same as above, just without normals...
	/*
	 roll = normalize(M_PI_2 - tf::Vector3(0, 1, 0).angle(tf::Vector3(0, b, c)));
	 pitch = normalize(
	 M_PI_2 - tf::Vector3(1, 0, 0).angle(tf::Vector3(a, 0, c)));
	 std::cout << "(roll, pitch)(2): " << roll << " " << pitch << std::endl;
	 */

	// yaw is the angle between the normal to the XZ-plane and the normal of the plane projected into the XZ-Plane
	// NOTE: we don't need yaw...
	tf::Vector3 normalXZ(0, 1, 0);
	yaw = normalize(normalXZ.angle(tf::Vector3(a, b, 0)));

	// third way...
	/*
	 tf::Vector3 v = tf::Vector3(a, b, c).cross(tf::Vector3(1, 0, 0));
	 tf::Vector3 u = v.cross(tf::Vector3(a, b, c));
	 roll = std::acos(v.normalized().dot(tf::Vector3(0, 1, 0)));
	 pitch = std::acos(u.normalized().dot(tf::Vector3(1, 0, 0)));
	 std::cout << "(roll, pitch)(3): " << roll << " " << pitch << std::endl;
	 */
}

void GroundData::setEquation(float a, float b, float c, float d) {
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
	normalizeEquation();
	/*
	std::cout << "setEquation() " << a << "," << b << "," << c << "," << d
			<< "\n";
	*/
	calculatePointsFromEquation();
}

double GroundData::normalize(double angle) {
	while (angle < -M_PI_2) {
		angle += M_PI;
	}

	while (angle > M_PI_2) {
		angle -= M_PI;
	}

	return angle;
}

void GroundData::calculatePointsFromEquation() {
	// points have to fulfill ax+by+cz+d=0 <=> ax+by+cz=-d

	// x=-d/a, y=z=0 fulfills the equation
	this->pointOne = tf::Vector3(-d / a, 0, 0);

	// y=-d/b, x=z=0 fulfills the equation
	this->pointTwo = tf::Vector3(0, -d / b, 0);

	// z=-d/c, x=y=0 fulfills the equation
	this->pointThree = tf::Vector3(0, 0, -d / c);

	/*
	std::cout << "c pointOne " << pointOne.x() << " " << pointOne.y() << " "
			<< pointOne.z() << " ";
	std::cout << "c pointTwo " << pointTwo.x() << " " << pointTwo.y() << " "
			<< pointTwo.z() << " ";
	std::cout << "c pointThree " << pointThree.x() << " " << pointThree.y()
			<< " " << pointThree.z() << "\n";
	*/
}

GroundData GroundData::transform(tf::Transform transform) const {
	GroundData gd;
	gd.pointOne = transform * this->pointOne;
	gd.pointTwo = transform * this->pointTwo;
	gd.pointThree = transform * this->pointThree;

	/*
	std::cout << "t pointOne " << pointOne.x() << " " << pointOne.y() << " "
			<< pointOne.z() << " ";
	std::cout << "t pointTwo " << pointTwo.x() << " " << pointTwo.y() << " "
			<< pointTwo.z() << " ";
	std::cout << "t pointThree " << pointThree.x() << " " << pointThree.y()
			<< " " << pointThree.z() << "\n";
	*/

	gd.calculateEquationFromPoints();
	return gd;
}

void GroundData::calculateEquationFromPoints() {
	// using the three points, solve the LGS and get a, b, c (fixing d=1)
	Eigen::Matrix3f A;
	Eigen::Vector3f b;
	A << pointOne.x(), pointOne.y(), pointOne.z(), pointTwo.x(), pointTwo.y(), pointTwo.z(), pointThree.x(), pointThree.y(), pointThree.z();
	b << -1, -1, -1;
	Eigen::Vector3f x = A.fullPivLu().solve(b);
	this->a = x[0];
	this->b = x[1];
	this->c = x[2];
	this->d = 1;
	normalizeEquation();
	/*
	std::cout << "calculateEquationFromPoints() " << this->a << "," << this->b
			<< "," << this->c << "," << this->d << "\n";
	*/
}

void GroundData::normalizeEquation() {
	double length = fabs(a) + fabs(b) + fabs(c) + fabs(d);
	a /= length;
	b /= length;
	c /= length;
	d /= length;
}

