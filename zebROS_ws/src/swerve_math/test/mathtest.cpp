#include <math.h>
#include "swerve_math/900Math.h"
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test");
    return RUN_ALL_TESTS();
}