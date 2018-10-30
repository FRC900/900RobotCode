#include <math.h>
#include "swerve_math/900Math.h"
#include <gtest/gtest.h>

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> Minor changes to tests
const double ERROR_MARGIN = 1e-4; // Error margin for floating comparisons

TEST(Math900, SignTests)
{
    EXPECT_EQ(-1, sign(-1e10));
    EXPECT_EQ(-1, sign(-1e-10));
    EXPECT_EQ(0, sign(0));
    EXPECT_EQ(1, sign(1e-10));
    EXPECT_EQ(1, sign(1e10));
}

<<<<<<< HEAD

TEST(Math900, NormAngle_LowerBound)
{
    EXPECT_TRUE(true);
}

TEST(Math900, NormAngle_UpperBound)
{
    EXPECT_NEAR(0, normalizeAngle(2 * M_PI), ERROR_MARGIN);
}

int main(int argc, char ** argv) 
{
    testing::InitGoogleTest(&argc, argv);
=======
int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test");
>>>>>>> Adding initial setup for GTest stuff
=======

TEST(Math900, NormAngle_LowerBound)
{
    EXPECT_TRUE(true);
}

TEST(Math900, NormAngle_UpperBound)
{
    EXPECT_NEAR(0, normalizeAngle(2 * M_PI), ERROR_MARGIN);
}

int main(int argc, char ** argv) 
{
    testing::InitGoogleTest(&argc, argv);
>>>>>>> Minor changes to tests
    return RUN_ALL_TESTS();
}