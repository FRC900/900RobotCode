
#ifndef GET_CONVERSION_FACTOR_INC__
#define GET_CONVERSION_FACTOR_INC__

#include "ctre_interfaces/talon_state_types.h"

double getConversionFactor(const int encoder_ticks_per_rotation,
                           const hardware_interface::FeedbackDevice encoder_feedback,
                           const hardware_interface::TalonMode talon_mode);

#endif