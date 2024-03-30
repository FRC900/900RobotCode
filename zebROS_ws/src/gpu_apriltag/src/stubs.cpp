#include <time.h>
#include "glog/logging.h"
#include "aos/time/time.h"

namespace aos
{
monotonic_clock::time_point monotonic_clock::now() noexcept {
#ifdef __linux__
  struct timespec current_time;
  PCHECK(clock_gettime(CLOCK_MONOTONIC, &current_time) == 0)
      << ": clock_gettime(" << static_cast<uintmax_t>(CLOCK_MONOTONIC) << ", "
      << &current_time << ") failed";

  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec));

#else  // __linux__

  __disable_irq();
  const uint32_t current_counter = SYST_CVR;
  uint32_t ms_count = systick_millis_count;
  const uint32_t istatus = SCB_ICSR;
  __enable_irq();
  // If the interrupt is pending and the timer has already wrapped from 0 back
  // up to its max, then add another ms.
  if ((istatus & SCB_ICSR_PENDSTSET) && current_counter > 50) {
    ++ms_count;
  }

  // It counts down, but everything we care about counts up.
  const uint32_t counter_up = ((F_CPU / 1000) - 1) - current_counter;

  // "3.2.1.2 System Tick Timer" in the TRM says "The System Tick Timer's clock
  // source is always the core clock, FCLK".
  using systick_duration =
      std::chrono::duration<uint32_t, std::ratio<1, F_CPU>>;

  return time_point(std::chrono::round<std::chrono::nanoseconds>(
      std::chrono::milliseconds(ms_count) + systick_duration(counter_up)));

#endif  // __linux__
}
}