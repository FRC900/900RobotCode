#ifndef INC_ARCHLEGTHTIME_TO_SEGTIME_H_
#define INC_ARCHLEGTHTIME_TO_SEGTIME_H_

#include <cmath>
#include <vector>

// Each spline segment is subdivided into multiple smaller arcs,
// picked such that these smaller arcs can be reasonably approximated
// by straight lines.  This function converts from an arc length
// fractional time into an integer segment time. The conversion is relatively
// simple except for the end condition...
template <class T>
size_t arcLengthTimeToSegTime(const std::vector<T> &equalArcLengthTimes, size_t i)
{
	// trajectory segments are still arbitrary times, each 1 unit long.
	// Thus, times 0 .. 0.99999 will be in segment 0, 1 .. 1.99999 in seg 1, and so on
	// The exception is the last time - the end time is at the end of segment t - 1
	size_t seg = std::floor(equalArcLengthTimes[i]);
	if (i == (equalArcLengthTimes.size() - 1))
		seg -= 1;
	return seg;
}

#endif
