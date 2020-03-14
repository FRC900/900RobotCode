cmake_minimum_required(VERSION 2.8)
set(CMAKE_SYSTEM_NAME Linux)
set(ARM_PREFIX arm-frc2020-linux-gnueabi)

set(CMAKE_C_COMPILER ${ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${ARM_PREFIX}-g++)

set(CMAKE_SYSROOT /home/ubuntu/wpilib/2020/roborio/${ARM_PREFIX})

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT};$ENV{HOME}/2020RobotCode/zebROS_ws/install_isolated)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(BOOST_ROOT ${ARM_PREFIX})
set(Boost_NO_SYSTEM_PATHS=ON)

find_program(CMAKE_RANLIB ${ARM_PREFIX}-gcc-ranlib)
find_program(CMAKE_AR ${ARM_PREFIX}-gcc-ar)
set(OPT_FLAGS "-O3 -flto=2 -fno-fat-lto-objects -mcpu=cortex-a9 -mfpu=neon -fvect-cost-model -ffunction-sections -fdata-sections -Wl,-gc-sections -Wno-psabi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_INSTALL_RPATH "$ENV{HOME}/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
