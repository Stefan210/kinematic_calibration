/*
 * ObservabilityIndex.h
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#ifndef OBSERVABILITYINDEX_H_
#define OBSERVABILITYINDEX_H_

namespace kinematic_calibration {

/**
 * Base class for the observability index of a set of poses.
 */
class ObservabilityIndex {
public:
	/**
	 * Constructor.
	 */
	ObservabilityIndex();

	/**
	 * Destructor.
	 */
	virtual ~ObservabilityIndex();
};

} /* namespace kinematic_calibration */

#endif /* OBSERVABILITYINDEX_H_ */
