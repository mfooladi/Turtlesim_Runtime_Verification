#ifndef TURTLE_RV_OBSTACLE_H
#define TURTLE_RV_OBSTACLE_H
#include <turtle_rv/ObstaclePoint.h>
#include <turtle_rv/ObstaclePosition.h>

#include <i2r_common/point.h>

namespace turtle_rv {
	bool operator==(const turtle_rv::ObstaclePosition & left, const turtle_rv::ObstaclePosition & right) {
		return left.position == right.position;
	}

	bool operator!=(const turtle_rv::ObstaclePosition & left, const turtle_rv::ObstaclePosition & right) {
		return left.position != right.position;
	}
}
#endif
