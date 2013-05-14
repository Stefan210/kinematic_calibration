/*
 * BallDetection.cpp
 *
 *  Created on: 01.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/BallDetection.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

BallDetection::BallData BallDetection::getPosition(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud) {
	BallDetection::BallData bd;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	removePlanes(initialCloud, cloudWithoutPlanes);
	segmentBall(cloudWithoutPlanes, cloudWithBallOnly, bd);

	this->cloudWithoutPlanes = cloudWithoutPlanes;
	this->cloudWithBallOnly = cloudWithBallOnly;
	this->ballPosition = bd.position;

	return bd;
}

BallDetection::BallData BallDetection::getAvgPosition(
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds) {
	float radiusSum = 0;
	float xSum = 0;
	float ySum = 0;
	float zSum = 0;
	BallDetection::BallData bd;

	for(int numOfCloud = 0; numOfCloud < clouds.size(); numOfCloud++) {
		BallDetection::BallData current = this->getPosition(clouds[numOfCloud]);
		xSum += current.position.x;
		ySum += current.position.y;
		zSum += current.position.z;
		radiusSum += current.radius;
	}

	bd.position.x = xSum / (float)clouds.size();
	bd.position.y = ySum / (float)clouds.size();
	bd.position.z = zSum / (float)clouds.size();
	bd.radius = radiusSum / (float)clouds.size();

	this->ballPosition = bd.position;

	return bd;
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

	//todo
	return true;
}

bool BallDetection::segmentBall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud,
		BallDetection::BallData& ballData) {
	pcl::SACSegmentation<pcl::PointXYZRGB> segSphere;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliersSphere = pcl::PointIndices::Ptr(
			new pcl::PointIndices());

	segSphere.setOptimizeCoefficients(false);
	segSphere.setModelType(pcl::SACMODEL_SPHERE); //detecting SPHERE
	segSphere.setMethodType(pcl::SAC_RANSAC);
	segSphere.setDistanceThreshold(0.01);
	segSphere.setRadiusLimits(this->minBallRadius, this->maxBallRadius); //0.01, 0.30
	segSphere.setMaxIterations(100000);
	segSphere.setInputCloud(inCloud);
	segSphere.segment(*inliersSphere, coefficients); //x,y,z,R

	ballData.position.x = coefficients.values[0];
	ballData.position.y = coefficients.values[1];
	ballData.position.z = coefficients.values[2];
	ballData.radius = coefficients.values[3];

	extract.setInputCloud(inCloud);
	extract.setIndices(inliersSphere);
	extract.setNegative(false);
	extract.filter(*outCloud);

	if (coefficients.values[3] <= this->maxBallRadius
			&& coefficients.values[3] >= this->minBallRadius) {
		return true;
	}
	return false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BallDetection::getCloudWithoutPlanes() {
	return this->cloudWithoutPlanes;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BallDetection::getCloudWithBallOnly() {
	return this->cloudWithBallOnly;
}

void BallDetection::setMinBallRadius(float minBallRadius) {
	this->minBallRadius = minBallRadius;
}
void BallDetection::setMaxBallRadius(float maxBallRadius) {
	this->maxBallRadius = maxBallRadius;
}
