/*
 * BallDetection.cpp
 *
 *  Created on: 02.05.2013
 *      Author: stefan
 */

#include "../include/BallDetection.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

BallDetection::BallDetection() {

}

BallDetection::~BallDetection() {

}

pcl::PointXYZRGB BallDetection::getPosition() {
	pcl::PointXYZRGB ballPosition;

	return ballPosition;
}

bool BallDetection::removePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {
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

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(inCloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(true);
	extract.filter(*outCloud);
	return false;
}
bool BallDetection::segmentBall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {
	pcl::SACSegmentation<pcl::PointXYZRGB> segSphere;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliersSphere = pcl::PointIndices::Ptr(
			new pcl::PointIndices());

	segSphere.setOptimizeCoefficients(false);
	segSphere.setModelType(pcl::SACMODEL_SPHERE); //detecting SPHERE
	segSphere.setMethodType(pcl::SAC_RANSAC);
	segSphere.setDistanceThreshold(0.01);
	segSphere.setRadiusLimits(0.01, 0.30);
	segSphere.setMaxIterations(100000);
	segSphere.setInputCloud(inCloud);
	segSphere.segment(*inliersSphere, coefficients);

	extract.setInputCloud(inCloud);
	extract.setIndices(inliersSphere);
	extract.setNegative(false);
	extract.filter(*outCloud);
	return false;
}
