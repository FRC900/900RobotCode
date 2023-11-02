///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \original author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser, Mrinal Kalakrishnan

#ifndef SIMPLE_SPLINE_SEGMENT_INC_
#define SIMPLE_SPLINE_SEGMENT_INC_

#include <array>
#include <stdexcept>
#include <trajectory_interface/trajectory_interface.h>
#include <spline_util/simple_pos_vel_acc_state.h>

namespace trajectory_interface
{

/**
 * \brief Class representing a multi-dimensional quintic spline segment with a start and end time.
 *
 * \tparam ScalarType Scalar type
 */
template<class ScalarType>
using SegmentState = SimplePosVelAccState<ScalarType>;

template<class ScalarType, size_t Order>
class SimpleSplineSegment
{
public:
  using Scalar = ScalarType;
  using Time = Scalar;
  using State = SimplePosVelAccState<Scalar>;
  using SplineCoefficients = std::array<Scalar, Order + 1>;

  /**
   * \brief Creates an empty segment.
   *
   * \note Calling <tt> size() </tt> on an empty segment will yield zero, and sampling it will yield a state with empty
   * data.
   */
  SimpleSplineSegment() = default;

  /**
   * \brief Construct segment from start and end states (boundary conditions).
   *
   * Please refer to the \ref init method documentation for the description of each parameter and the exceptions that
   * can be thrown.
   */
  SimpleSplineSegment(const Time&  start_time,
                      const State& start_state,
                      const Time&  end_time,
                      const State& end_state)
  {
    init(start_time, start_state, end_time, end_state);
  }

  /**
   * \brief Initialize segment from start and end states (boundary conditions).
   *
   * The start and end states need not necessarily be specified all the way to the acceleration level:
   * - If only \b positions are specified, linear interpolation will be used.
   * - If \b positions and \b velocities are specified, a cubic spline will be used.
   * - If \b positions, \b velocities and \b accelerations are specified, a quintic spline will be used.
   *
   * \note If start and end states have different specifications
   * (eg. start is positon-only, end is position-velocity), the lowest common specification will be used
   * (position-only in the example).
   *
   * \param start_time Time at which the segment state equals \p start_state.
   * \param start_state State at \p start_time.
   * \param end_time Time at which the segment state equals \p end_state.
   * \param end_state State at time \p end_time.
   *
   * \throw std::invalid_argument If the \p end_time is earlier than \p start_time or if one of the states is
   * uninitialized.
   */
  void init(const Time&  start_time,
            const State& start_state,
            const Time&  end_time,
            const State& end_state)
  {
    // Preconditions
    if (end_time < start_time)
    {
      throw(std::invalid_argument("Quintic spline segment can't be constructed: end_time < start_time."));
    }
    // Time data
    start_time_ = start_time;
    duration_   = end_time - start_time;

    computeCoefficients(start_state.position, start_state.velocity, start_state.acceleration,
					    end_state.position,   end_state.velocity,   end_state.acceleration,
					    duration_,
					    coefs_);
  }

  /**
   * \brief Sample the segment at a specified time.
   *
   * \note Within the <tt>[start_time, end_time]</tt> interval, spline interpolation takes place, outside it this method
   * will output the start/end states with zero velocity and acceleration.
   *
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p time.
   */
  void sample(const Time& time, State& state) const
  {
      sampleWithTimeBounds(coefs_,
                           duration_, time - start_time_,
                           state.position, state.velocity, state.acceleration);
  }

  /**
   * \brief Sample the segment position at a specified time.
   *
   * \note Within the <tt>[start_time, end_time]</tt> interval, spline interpolation takes place, outside it this method
   * will output start/end position.
   *
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p time.
   */
  void samplePosition(const Time& time, Scalar& position) const
  {
      samplePositionWithTimeBounds(coefs_,
                           duration_, time - start_time_,
                           position);
  }

  /**
   * \brief Sample the segment velocity at a specified time.
   *
   * \note Within the <tt>[start_time, end_time]</tt> interval, spline interpolation takes place, outside it this method
   * will output zero velocity.
   *
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p time.
   */
  void sampleVelocity(const Time& time, Scalar& velocity) const
  {
      sampleVelocityWithTimeBounds(coefs_,
                           duration_, time - start_time_,
                           velocity);
  }

  /** \return Segment start time. */
  Time startTime() const {return start_time_;}

  /** \return Segment end time. */
  Time endTime() const {return start_time_ + duration_;}

  /** \return Spline Coefficients. */
  const SplineCoefficients &getCoefs(void) const { return coefs_; }

  /** \return Segment size (dimension). */
  constexpr unsigned int size() const {return 1;}

private:

  /** Coefficients represent a quintic polynomial like so:
   *
   * <tt> coefs_[0] + coefs_[1]*x + coefs_[2]*x^2 + coefs_[3]*x^3 + coefs_[4]*x^4 + coefs_[5]*x^5 </tt>
   */
  SplineCoefficients coefs_{};
  Time duration_{static_cast<Scalar>(0)};
  Time start_time_{static_cast<Scalar>(0)};

  // These methods are borrowed from the previous controller's implementation
  static void generatePowers(const Scalar& x, std::array<Scalar, Order> &powers)
  {
    powers[0] = x;
    for (size_t i = 1; i < Order; ++i)
    {
      powers[i] = powers[i - 1]*x;
    }
  }

  // Specialization for Linear (order 1) splines
  template <size_t O = Order>
  static typename std::enable_if_t<O == 1, void>
  computeCoefficients(const Scalar& start_pos, const Scalar& /*start_vel*/, const Scalar& /*start_acc*/,
					  const Scalar& end_pos,   const Scalar& /*end_vel*/,   const Scalar& /*end_acc*/,
					  const Scalar& time,
					  SplineCoefficients& coefficients)
  {
    coefficients[0] = start_pos;
    coefficients[1] = (time == 0.0) ? 0.0 : (end_pos - start_pos) / time;
  }

  static void sample(const SplineCoefficients& coefficients, const Scalar& time,
		 Scalar& position, Scalar& velocity, Scalar& acceleration)
  {
	// Calls specialized functions below based on spline order
	samplePosition(coefficients, time, position);
	sampleVelocity(coefficients, time, velocity);
	sampleAcceleration(coefficients, time, acceleration);
  }

  // Specalization for Linear (order 1) splines
  template <size_t O = Order>
  static typename std::enable_if_t<O == 1, void>
  samplePosition(const SplineCoefficients& coefficients, const Scalar& time, Scalar& position)
  {
    position =      coefficients[0] +
			   time*coefficients[1];
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 1, void>
  sampleVelocity(const SplineCoefficients& coefficients, const Scalar& time, Scalar& velocity)
  {
	static_cast<void>(time);
    velocity = coefficients[1];
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 1, void>
  sampleAcceleration(const SplineCoefficients& coefficients, const Scalar& time, Scalar& acceleration)
  {
	static_cast<void>(coefficients);
	static_cast<void>(time);
	acceleration = static_cast<Scalar>(0);
  }


  // Specializations for Cubic (order 3) splines
  template <size_t O = Order>
  static typename std::enable_if_t<O == 3, void>
  computeCoefficients(const Scalar& start_pos, const Scalar& start_vel, const Scalar& /*start_acc*/,
					  const Scalar& end_pos,   const Scalar& end_vel,   const Scalar& /*end_acc*/,
					  const Scalar& time,
					  SplineCoefficients& coefficients)
  {
    if (time == 0.0)
    {
	  coefficients[0] = start_pos;
	  coefficients[1] = start_vel;
	  coefficients[2] = 0.0;
	  coefficients[3] = 0.0;
    }
    else
    {
    std::array<Scalar, O> T;
	  generatePowers(time, T);

	  coefficients[0] = start_pos;
	  coefficients[1] = start_vel;
	  coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[0] - end_vel*T[0]) / T[1];
	  coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[0] + end_vel*T[0]) / T[2];
    }
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 3, void>
  samplePosition(const SplineCoefficients& coefficients, const Scalar& time, Scalar& position)
  {
	position = ((coefficients[3] * time + coefficients[2]) * time + coefficients[1]) * time + coefficients[0];
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 3, void>
  sampleVelocity(const SplineCoefficients& coefficients, const Scalar& time, Scalar& velocity)
  {
	velocity = (Scalar(3.0) * coefficients[3] * time + coefficients[2]) * time + coefficients[1];
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 3, void>
  sampleAcceleration(const SplineCoefficients& coefficients, const Scalar& time, Scalar& acceleration)
  {
	acceleration =
			Scalar(2.0)*     coefficients[2] +
		    Scalar(6.0)*time*coefficients[3];
  }

  // Specialization for Qunitic (order 5) splines
  template <size_t O = Order>
  static typename std::enable_if_t<O == 5, void>
  computeCoefficients(const Scalar& start_pos, const Scalar& start_vel, const Scalar& start_acc,
					  const Scalar& end_pos,   const Scalar& end_vel,   const Scalar& end_acc,
					  const Scalar& time,
					  SplineCoefficients& coefficients)
  {
    if (time == 0.0)
    {
	  coefficients[0] = start_pos;
	  coefficients[1] = start_vel;
	  coefficients[2] = 0.5*start_acc;
	  coefficients[3] = 0.0;
	  coefficients[4] = 0.0;
	  coefficients[5] = 0.0;
    }
    else
    {
    std::array<Scalar, O> T;
	  generatePowers(time, T);

	  coefficients[0] = start_pos;
	  coefficients[1] = start_vel;
	  coefficients[2] = 0.5*start_acc;
	  coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[1] + end_acc*T[1] -
					     12.0*start_vel*T[0] - 8.0*end_vel*T[0]) / (2.0*T[2]);
	  coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[1] - 2.0*end_acc*T[1] +
					     16.0*start_vel*T[0] + 14.0*end_vel*T[0]) / (2.0*T[3]);
	  coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[1] -
					     6.0*start_vel*T[0] - 6.0*end_vel*T[0]) / (2.0*T[4]);
    }
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 5, void>
  samplePosition(const SplineCoefficients& coefficients, const Scalar& time, Scalar& position)
  {
	position = ((((time * coefficients[5] + coefficients[4]) *
				   time + coefficients[3]) *
				   time + coefficients[2]) *
				   time + coefficients[1]) *
				   time + coefficients[0];
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 5, void>
  sampleVelocity(const SplineCoefficients& coefficients, const Scalar& time, Scalar& velocity)
  {
	velocity = (((time * Scalar(5.0) * coefficients[5] + Scalar(4.0) * coefficients[4]) *
				  time + Scalar(3.0) * coefficients[3]) *
				  time + Scalar(2.0) * coefficients[2]) *
				  time + coefficients[1];
  }

  template <size_t O = Order>
  static typename std::enable_if_t<O == 5, void>
  sampleAcceleration(const SplineCoefficients& coefficients, const Scalar& time, Scalar& acceleration)
  {
	acceleration = ((time * Scalar(20.0) * coefficients[5] + Scalar(12.0) * coefficients[4]) *
				   time + Scalar(6.0) * coefficients[3]) *
				   time + Scalar(2.0) * coefficients[2];
  }

  static void sampleWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& duration, const Scalar& time,
								   Scalar& position, Scalar& velocity, Scalar& acceleration)
  {
    if (time < 0)
    {
      samplePosition(coefficients, 0.0, position);
      velocity = 0;
      acceleration = 0;
    }
    else if (time > duration)
    {
      samplePosition(coefficients, duration, position);
      velocity = 0;
      acceleration = 0;
    }
    else
    {
      sample(coefficients, time,
             position, velocity, acceleration);
    }
  }

  static void samplePositionWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& duration, const Scalar& time,
								   Scalar& position)
  {
    if (time < 0)
    {
      samplePosition(coefficients, 0.0, position);
    }
    else if (time > duration)
    {
      samplePosition(coefficients, duration, position);
    }
    else
    {
      samplePosition(coefficients, time, position);
    }
  }
  static void sampleVelocityWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& duration, const Scalar& time,
								   Scalar& velocity)
  {
    if (time < 0)
    {
      velocity = 0;
    }
    else if (time > duration)
    {
      velocity = 0;
    }
    else
    {
      sampleVelocity(coefficients, time, velocity);
    }
  }
};

} // namespace

namespace trajectory_interface
{

/**
 * \brief Sample a trajectory position at a specified time.
 *
 * This is a convenience function that combines finding the segment associated to a specified time (see \ref findSegment)
 * and sampling it.
 *
 * \tparam Trajectory Trajectory type. Should be a \e sequence container \e sorted by segment start time.
 * \param[in] trajectory Holds a sequence of segments.
 * \param[in] time Where the trajectory is to be sampled.
 * \param[out] position Segment position at \p time.
 *
 * \return Iterator to the trajectory segment containing \p time. If no segment contains \p time, then
 * <tt>trajectory.end()</tt> is returned.
 *
 * \note The segment implementation should allow sampling outside the trajectory time interval, implementing a policy
 * such as holding the first/last position. In such a case, it is possible to get a valid sample for all time inputs,
 * even for the cases when this function returns <tt>trajectory.end()</tt>.
 *
 * \sa findSegment
 */
template<class Trajectory>
inline typename Trajectory::const_iterator samplePosition(const Trajectory&                             trajectory,
                                                  const typename Trajectory::value_type::Time&  time,
                                                        typename Trajectory::value_type::Scalar& position)
{
  typename Trajectory::const_iterator it = findSegment(trajectory, time);
  if (it != trajectory.end())
  {
    it->samplePosition(time, position); // Segment found at specified time
  }
  else if (!trajectory.empty())
  {
    trajectory.front().samplePosition(time, position); // Specified time preceeds trajectory start time
  }
  else
  {
	  position = 0;
  }
  return it;
}

/**
 * \brief Sample a trajectory velocity at a specified time.
 *
 * This is a convenience function that combines finding the segment associated to a specified time (see \ref findSegment)
 * and sampling it.
 *
 * \tparam Trajectory Trajectory type. Should be a \e sequence container \e sorted by segment start time.
 * \param[in] trajectory Holds a sequence of segments.
 * \param[in] time Where the trajectory is to be sampled.
 * \param[out] position Segment position at \p time.
 *
 * \return Iterator to the trajectory segment containing \p time. If no segment contains \p time, then
 * <tt>trajectory.end()</tt> is returned.
 *
 * \note The segment implementation should allow sampling outside the trajectory time interval, implementing a policy
 * such as holding the first/last position. In such a case, it is possible to get a valid sample for all time inputs,
 * even for the cases when this function returns <tt>trajectory.end()</tt>.
 *
 * \sa findSegment
 */
template<class Trajectory>
inline typename Trajectory::const_iterator sampleVelocity(const Trajectory&                             trajectory,
                                                  const typename Trajectory::value_type::Time&  time,
                                                        typename Trajectory::value_type::Scalar& velocity)
{
  typename Trajectory::const_iterator it = findSegment(trajectory, time);
  if (it != trajectory.cend())
  {
    it->sampleVelocity(time, velocity); // Segment found at specified time
  }
  else if (!trajectory.empty())
  {
    trajectory.front().sampleVelocity(time, velocity); // Specified time preceeds trajectory start time
  }
  else
  {
	  velocity = 0;
  }
  return it;
}

} // namespace

#endif
