# Various compiler optimizations to apply to all nodes

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-Wno-deprecated-declarations -Wno-switch -ftrack-macro-expansion=0 -fno-var-tracking-assignments -DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS -Wall -DNON_POLLING)
#add_definitions(-Wall -Wextra -Wno-switch)

if (DEFINED CMAKE_TOOLCHAIN_FILE)  # Cross-build for Rio
  # Everything is in the toolchain file
else() # Native builds
  set (CMAKE_RANLIB "gcc-ranlib" )
  set (CMAKE_AR     "gcc-ar"     )
  
  set (OPT_FLAGS "${OPT_FLAGS} -Ofast -fno-finite-math-only -flto=2 -fno-fat-lto-objects -ffunction-sections -fdata-sections -Wl,-gc-sections")
  if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (OPT_FLAGS "${OPT_FLAGS} -mcpu=cortex-a15 -mfpu=neon-vfpv4 -fvect-cost-model")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2
	set (OPT_FLAGS "${OPT_FLAGS} -march=armv8-a+crypto -mcpu=cortex-a57+crypto -fvect-cost-model")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  else() # x86? Mac?
    set (OPT_FLAGS "${OPT_FLAGS} -march=native -mtune=native")
  endif()
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS} -fuse-linker-plugin")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS} -fuse-linker-plugin")

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

