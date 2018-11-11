#include <math.h>
#include "swerve_math/900Math.h"
#include <gtest/gtest.h>

const double ERROR_MARGIN = 1e-4; // Error margin for floating comparisons

TEST(Math900, SignTests)
{
    EXPECT_EQ(-1, sign(-1e10));
    EXPECT_EQ(-1, sign(-1e-10));
    EXPECT_EQ(0, sign(0));
    EXPECT_EQ(1, sign(1e-10));
    EXPECT_EQ(1, sign(1e10));
}


TEST(Math900, NormAngle_LowerBound)
{
    EXPECT_TRUE(true);
}

TEST(Math900, NormAngle_UpperBound)
{
    EXPECT_NEAR(0, normalizeAngle(2 * M_PI), ERROR_MARGIN);
}

TEST(Math900, NormAngle_UpperBound)
{
    EXPECT_NEAR(0, normalizeAngle(2 * M_PI), ERROR_MARGIN);
}

int main(int argc, char ** argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}