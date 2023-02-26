#include <cmath>

double getFourBarLength(double angle, double min_extension, double parallel_bar_length, double diagonal_bar_length, double intake_length)
{
    double minAngle = acos((min_extension - intake_length - parallel_bar_length) / diagonal_bar_length);
    // if below, angle = minAngle + acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_). if above, angle = minAngle - acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_).
    if (angle > minAngle) {
        return cos(angle - minAngle) * diagonal_bar_length + intake_length + parallel_bar_length;
    } else {
        // (minAngle - (minAngle - angle)) = minAngle - minAngle + angle = angle
        return cos(minAngle - angle) * diagonal_bar_length + intake_length + parallel_bar_length;
    }
}