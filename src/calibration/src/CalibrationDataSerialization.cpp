/*
 * CalibrationDataSerialization.cpp
 *
 *  Created on: 03.07.2013
 *      Author: stefan
 */

#include "../include/CalibrationDataSerialization.h"

#include <iostream>
#include <fstream>

CalibrationDataSerialization::CalibrationDataSerialization(std::string filename) :
		dataLoaded(false), dataSaved(false) {
	this->fileName = filename;
}

CalibrationDataSerialization::~CalibrationDataSerialization() {

}

void CalibrationDataSerialization::optimizeTransform(
		tf::Transform& cameraToHead) {
	saveToFile();
}

std::vector<MeasurePoint> CalibrationDataSerialization::getMeasurementSeries() {
	if(!dataLoaded) {
		loadFromFile();
	}
	return this->measurePoints;
}

tf::Transform CalibrationDataSerialization::getInitialTransform() {
	if(!dataLoaded) {
		loadFromFile();
	}
	return this->initialTransformCameraToHead;
}

void CalibrationDataSerialization::saveToFile() {
	cout << "writing data to file " << fileName << endl;
	ofstream file;
	file.open(fileName.c_str());
	file << " " << this->measurePoints.size();
	for (int i = 0; i < this->measurePoints.size(); i++) {
		file << " " << this->measurePoints[i];
	}
	file << " " << this->initialTransformCameraToHead;
	file.close();
	dataSaved = true;
}

void CalibrationDataSerialization::loadFromFile() {
	ifstream file;
	file.open(fileName.c_str());
	int size;
	file >> size;
	for (int i = 0; i < size; i++) {
		MeasurePoint current;
		file >> current;
		this->measurePoints.push_back(current);
	}

	file >> this->initialTransformCameraToHead;
	file.close();
	dataLoaded = true;
}

