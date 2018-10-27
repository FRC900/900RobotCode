#include <math.h>
#include "swerve_math/900Math.h"
#include <gtest/gtest.h>
#include <gtest/gmock.h>

TEST(x, y)
{
    EXPECT_EQ(2, 2);
}

TEST(DummyTest, Dummy)
{
    ASSERT_EQ(3, 3);
}

int main(int argc, char ** argv) 
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test");
    return RUN_ALL_TESTS();
}