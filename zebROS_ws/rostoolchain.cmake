cmake_minimum_required(VERSION 3.0)
set(CMAKE_SYSTEM_NAME Linux)
set(ARM_PREFIX arm-frc2023-linux-gnueabi)

set(CMAKE_C_COMPILER ${ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${ARM_PREFIX}-g++)

set(CMAKE_SYSROOT $ENV{HOME}/wpilib/2023/roborio/arm-nilrt-linux-gnueabi/sysroot)

# Set find_libraries and friends to only look in sysroot, ignoring host lib dirs
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT};$ENV{HOME}/2022RobotCode/zebROS_ws/install_isolated)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Similar settings as above, except for pkg_check_modules()
set(ENV{PKG_CONFIG_DIR} "")
set(ENV{PKG_CONFIG_PATH} "")
set(ENV{PKG_CONFIG_LIBDIR} "${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})

set(BOOST_ROOT ${ARM_PREFIX})
set(Boost_NO_SYSTEM_PATHS=ON)

find_program(CMAKE_RANLIB ${ARM_PREFIX}-gcc-ranlib)
find_program(CMAKE_AR ${ARM_PREFIX}-gcc-ar)
set(OPT_FLAGS "-O3 -flto=auto -fno-fat-lto-objects -fuse-linker-plugin -mcpu=cortex-a9 -mfpu=neon -fvect-cost-model -ffunction-sections -fdata-sections -Wl,-gc-sections -Wno-psabi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_INSTALL_RPATH "$ENV{HOME}/wpilib/2023/roborio/arm-nilrt-linux-gnueabi/sysroot/opt/ros/melodic/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
