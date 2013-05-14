/*
 * BallDetection.h
 *
 *  Created on: 01.05.2013
 *      Author: Stefan Wrobel
 */

#ifndef BALLDETECTION_H_
#define BALLDETECTION_H_

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Default parameter
#define MIN_BALL_RADIUS (0.05)
#define MAX_BALL_RADIUS (0.10)

/**
 * Class for detecting a ball contained in a point cloud.
 */
class BallDetection {

public:
	struct BallData {
		pcl::PointXYZ position;
		float radius;
	};

	/// Constructor
	BallDetection() :
			minBallRadius(MIN_BALL_RADIUS), maxBallRadius(MAX_BALL_RADIUS) {
	}

	/// Deconstructor
	~BallDetection() {
	}

	/**
	 * Given a point cloud containing a ball (general: sphere), tries
	 * to segment it and return its position (center).
	 * @param initialCloud The point cloud containing the ball.
	 * @return The position of the detected ball.
	 */
	BallData getPosition(
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud);

	BallData getAvgPosition(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

	/**
	 * Returns the cloud after extracting the planes.
	 * Should only be called after a call to getPosition().
	 * @return The cloud after extracting the planes.
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudWithoutPlanes();

	/**
	 * Returns the cloud which contains only the ball.
	 * Should only be called after a call to getPosition().
	 * @return The cloud which contains only the ball.
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudWithBallOnly();

	/**
	 * Sets a lower bound for the ball radius.
	 * @param minBallRadius Lower bound for the ball radius.
	 */
	void setMinBallRadius(float minBallRadius);

	/**
	 * Sets an upper bound for the ball radius.
	 * @param maxBallRadius Upper bound for the ball radius.
	 */
	void setMaxBallRadius(float maxBallRadius);

protected:
	/**
	 * Extracts planes (e.g. floor) contained in the point cloud.
	 * @param inCloud Point cloud which contains planes.
	 * @param outCloud Point cloud with planes being extracted.
	 * @return "True" if successful, "False" otherwise.
	 */
	bool removePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);

	/**
	 * Segmentation of the ball.
	 * @param inCloud Initial point cloud (should contain exactly one sphere).
	 * @param outCloud Point cloud containing only the ball.
	 * @param ballPosition Position of the ball which was found.
	 * @return "True" if successful, "False" otherwise.
	 */
	bool segmentBall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud,
			BallDetection::BallData& ballData);

private:
	/**
	 * Holds the position of the detected ball.
	 */
	pcl::PointXYZ ballPosition;

	/**
	 * Holds a reference to the point cloud after extracting the planes.
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes;

	/**
	 * Holds a reference to the point cloud which containts only the ball.
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly;

	/**
	 * Minimum ball radius.
	 */
	float minBallRadius;

	/**
	 * Maximum ball radius.
	 */
	float maxBallRadius;

};

#endif /* BALLDETECTION_H_ */
