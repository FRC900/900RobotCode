# Tries to find Gperftools.
#
# Usage of this module as follows:
#
# find_package(Gperftools)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# Gperftools_ROOT_DIR  Set this variable to the root installation of Gperftools
# if the module has problems finding the proper installation path.
#
# Variables defined by this module:
#
# GPERFTOOLS_FOUND              System has Gperftools libs/headers
# GPERFTOOLS_LIBRARIES          The Gperftools libraries (tcmalloc & profiler)
# GPERFTOOLS_INCLUDE_DIR        The location of Gperftools headers

find_library(
  GPERFTOOLS_TCMALLOC
  NAMES tcmalloc
  HINTS ${Gperftools_ROOT_DIR}/lib)

# GetStackTrace(void**, int, int) was causing a segfault during ros::init --> boost calling tcalloc
# So, if we don't track stack traces (and use the minimal version) then this segfault doesn't happen
# Inspiration from https://github.com/gperftools/gperftools/issues/1159
# Also, a note about where this is being called: https://github.com/gperftools/gperftools/blob/a81b2ebbc2cec046aed5d571cdc783c49b48843a/src/page_heap.cc#L727 locking context --> HandleUnlock --> GrabBacktrace --> GetStackTrace https://github.com/gperftools/gperftools/blob/a81b2ebbc2cec046aed5d571cdc783c49b48843a/src/malloc_backtrace.cc#L78
find_library(
  GPERFTOOLS_TCMALLOC_MINIMAL
  NAMES tcmalloc_minimal
  HINTS ${Gperftools_ROOT_DIR}/lib)

find_library(
  GPERFTOOLS_PROFILER
  NAMES profiler
  HINTS ${Gperftools_ROOT_DIR}/lib)

find_library(
  GPERFTOOLS_TCMALLOC_AND_PROFILER
  NAMES tcmalloc_and_profiler
  HINTS ${Gperftools_ROOT_DIR}/lib)

find_path(
  GPERFTOOLS_INCLUDE_DIR
  NAMES gperftools/heap-profiler.h
  HINTS ${Gperftools_ROOT_DIR}/include)

set(GPERFTOOLS_LIBRARIES ${GPERFTOOLS_TCMALLOC_MINIMAL})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gperftools DEFAULT_MSG GPERFTOOLS_LIBRARIES
                                  GPERFTOOLS_INCLUDE_DIR)

mark_as_advanced(
  Gperftools_ROOT_DIR GPERFTOOLS_TCMALLOC GPERFTOOLS_PROFILER
  GPERFTOOLS_TCMALLOC_AND_PROFILER GPERFTOOLS_LIBRARIES GPERFTOOLS_INCLUDE_DIR)

# create IMPORTED targets
if (Gperftools_FOUND AND NOT TARGET gperftools::tcmalloc)
  add_library(gperftools::tcmalloc UNKNOWN IMPORTED)
  set_target_properties(
    gperftools::tcmalloc
    PROPERTIES
      IMPORTED_LOCATION ${GPERFTOOLS_TCMALLOC} INTERFACE_INCLUDE_DIRECTORIES
                                               "${GPERFTOOLS_INCLUDE_DIR}")
  add_library(gperftools::profiler UNKNOWN IMPORTED)
  set_target_properties(
    gperftools::profiler
    PROPERTIES
      IMPORTED_LOCATION ${GPERFTOOLS_PROFILER} INTERFACE_INCLUDE_DIRECTORIES
                                               "${GPERFTOOLS_INCLUDE_DIR}")
endif ()

