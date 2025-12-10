#include <gtest/gtest.h>

#include "LieGroup/LieGroup.h"

namespace srlib::tests {

TEST(LieGroupSmokeTest, IdentityTransform)
{
  SE3 identity = SE3();
  Vec3 v{1.0, 2.0, 3.0};
  Vec3 out = identity * v;
  EXPECT_DOUBLE_EQ(out[0], v[0]);
  EXPECT_DOUBLE_EQ(out[1], v[1]);
  EXPECT_DOUBLE_EQ(out[2], v[2]);
}

TEST(LieGroupSmokeTest, InverseTransform)
{
  SO3 rot = Exp(Vec3(0.1, 0.2, -0.1));
  SE3 pose(rot, Vec3(0.5, -0.3, 0.8));

  Vec3 point{0.4, 0.2, -0.1};
  Vec3 mapped = pose * point;
  Vec3 recovered = Inv(pose) * mapped;

  EXPECT_NEAR(recovered[0], point[0], 1e-12);
  EXPECT_NEAR(recovered[1], point[1], 1e-12);
  EXPECT_NEAR(recovered[2], point[2], 1e-12);
}

}  // namespace srlib::tests
