/*
 * JointUpdate.h
 *
 *  Created on: 22.11.2013
 *      Author: stefan
 */

#ifndef JOINTUPDATE_H_
#define JOINTUPDATE_H_

#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <map>
#include <string>
#include <vector>

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class JointUpdate {

public:
	JointUpdate(urdf::Model model, string prefix);
	virtual ~JointUpdate();

	void setOffsets(map<string, double> offsets);

	void getModifiedJoints(vector<urdf::Joint>& joints);

	void writeCalibrationData(const string& filename);

	void writeCalibrationData(const map<string, double> offsets,
			const string& filename);

private:
	urdf::Model model;
	map<string, double> offsets;
	string prefix;
};

} /* namespace kinematic_calibration */

#endif /* JOINTUPDATE_H_ */
