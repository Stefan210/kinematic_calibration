/*
 * CheckerboardContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CHECKERBOARDCONTEXT_H_
#define CHECKERBOARDCONTEXT_H_

#include "../data_capturing/CheckerboardDetection.h"
#include "MarkerContext.h"

namespace kinematic_calibration {

/*
 *
 */
class CheckerboardContext: public MarkerContext {
public:
	CheckerboardContext();
	virtual ~CheckerboardContext();

	virtual MarkerDetection* getMarkerDetectionInstance() {
		return new CheckerboardDetection();
	}
};

} /* namespace kinematic_calibration */

#endif /* CHECKERBOARDCONTEXT_H_ */
