/*
 * AbstractContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef ABSTRACTCONTEXT_H_
#define ABSTRACTCONTEXT_H_

#include "../data_capturing/MarkerDetection.h"

namespace kinematic_calibration {

/*
 *
 */
class AbstractContext {
public:
	AbstractContext();
	virtual ~AbstractContext();

	virtual MarkerDetection* getMarkerDetectionInstance() = 0;
};

} /* namespace kinematic_calibration */

#endif /* ABSTRACTCONTEXT_H_ */
