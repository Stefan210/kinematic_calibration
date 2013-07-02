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
	segmentPlane(cloudDistanceFiltered, gd);

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
	segPlane.setDistanceThreshold(0.03);
	segPlane.setInputCloud(inCloud);
	segPlane.setInputNormals(cloud_normals);

	// Obtain the plane inliers and coefficients
	segPlane.segment(*inliers_plane, *coefficients_plane);

	gd.a = coefficients_plane->values[0];
	gd.b = coefficients_plane->values[1];
	gd.c = coefficients_plane->values[2];
	gd.d = coefficients_plane->values[3];
}

const tf::Pose GroundData::getPose() const {
	float roll, pitch, yaw;
	getRPY(roll, pitch, yaw);
	tf::Quaternion q; q.setRPY(roll, pitch, yaw);
	tf::Pose pose(q, tf::Vector3(0, 0, 0));
	return pose;
}

void GroundData::getRPY(float& roll, float& pitch, float& yaw) const {
	tf::Vector3 normalXY(0,0,1);
	roll = normalXY.angle(tf::Vector3(0,b,c));
	pitch = normalXY.angle(tf::Vector3(a,0,c));

	tf::Vector3 normalXZ(0,1,0);
	yaw = normalXZ.angle(tf::Vector3(a,b,0));
}



