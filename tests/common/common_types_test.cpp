#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <type_traits>

#include "common/types.h"

namespace srlib::tests {

TEST(CommonTypes, RealAliasAndConstants)
{
  static_assert(std::is_same_v<real, double>, "real should be a double");

  EXPECT_DOUBLE_EQ(SR_ZERO, 0.0);
  EXPECT_DOUBLE_EQ(SR_HALF, 0.5);
  EXPECT_DOUBLE_EQ(SR_ONE, 1.0);
  EXPECT_DOUBLE_EQ(SR_TWO, 2.0);

  EXPECT_NEAR(SR_PI_HALF * 2.0, SR_PI, 1e-12);
  EXPECT_NEAR(SR_TWO_PI, SR_PI * 2.0, 1e-12);
  EXPECT_NEAR(SR_PI_SQRT2 * std::sqrt(2.0), SR_PI, 1e-12);
  EXPECT_NEAR(SR_ONETHIRD * 3.0, 1.0, 1e-12);
  EXPECT_NEAR(SR_ONESIXTH * 6.0, 1.0, 1e-12);
  EXPECT_NEAR(SR_FOURTHIRD * 3.0, 4.0, 1e-12);
  EXPECT_NEAR(SR_RADIAN * 180.0, SR_PI, 1e-12);
  EXPECT_NEAR(SR_DEGREE, 1.0 / SR_RADIAN, 1e-9);
}

TEST(CommonTypes, DegRadConversions)
{
  EXPECT_NEAR(DEG2RAD(180), SR_PI, 1e-12);
  EXPECT_NEAR(RAD2DEG(SR_PI), 180.0, 1e-12);

  EXPECT_NEAR(DEG2RAD(90.0f), SR_PI_HALF, 1e-12);
  EXPECT_NEAR(RAD2DEG(SR_PI_HALF), 90.0, 1e-12);

  EXPECT_NEAR(DEG2RAD(45.0), SR_PI / 4.0, 1e-12);
  EXPECT_NEAR(RAD2DEG(SR_PI / 4.0), 45.0, 1e-12);
}

TEST(CommonTypes, ComparisonsAndRound)
{
  EXPECT_TRUE(SR_ISZERO(5e-7));
  EXPECT_FALSE(SR_ISZERO(5e-5));

  EXPECT_TRUE(SR_ISEQUAL(1.0, 1.0 + 5e-7));
  EXPECT_FALSE(SR_ISEQUAL(1.0, 1.0 + 2e-5));

  EXPECT_DOUBLE_EQ(SR_ROUND(2.3), 2.0);
  EXPECT_DOUBLE_EQ(SR_ROUND(2.7), 3.0);
  EXPECT_DOUBLE_EQ(SR_ROUND(-1.2), -1.0);
  EXPECT_DOUBLE_EQ(SR_ROUND(-1.8), -2.0);
}

TEST(CommonTypes, RandomRange)
{
  srand(42);
  for (int i = 0; i < 10; ++i) {
    const double value = SR_RAND(-3.0, 7.0);
    EXPECT_GE(value, -3.0);
    EXPECT_LE(value, 7.0);
  }
}

}  // namespace srlib::tests
