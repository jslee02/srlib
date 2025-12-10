#include <gtest/gtest.h>

#include "common/utils.h"

namespace srlib::tests {

class DestroyableMock {
public:
  explicit DestroyableMock(bool& destroyed_flag) : destroyed(destroyed_flag) {}

  void DestroyWindow() { destroyed = true; }

private:
  bool& destroyed;
};

class ReleasableMock {
public:
  explicit ReleasableMock(bool& released_flag) : released(released_flag) {}

  void Release() { released = true; }

private:
  bool& released;
};

TEST(CommonUtils, SafeDelete)
{
  int* value = new int(5);
  SR_SAFE_DELETE(value);
  EXPECT_EQ(value, nullptr);

  SR_SAFE_DELETE(value);
  EXPECT_EQ(value, nullptr);
}

TEST(CommonUtils, SafeDeleteArray)
{
  int* values = new int[3]{1, 2, 3};
  SR_SAFE_DELETE_AR(values);
  EXPECT_EQ(values, nullptr);

  SR_SAFE_DELETE_AR(values);
  EXPECT_EQ(values, nullptr);
}

TEST(CommonUtils, SafeDestroyWindow)
{
  bool destroyed = false;
  DestroyableMock* window = new DestroyableMock(destroyed);

  SR_SAFE_DESTROY_WINDOW(window);
  EXPECT_TRUE(destroyed);
  EXPECT_EQ(window, nullptr);

  SR_SAFE_DESTROY_WINDOW(window);
  EXPECT_TRUE(destroyed);
  EXPECT_EQ(window, nullptr);
}

TEST(CommonUtils, SafeRelease)
{
  bool released = false;
  ReleasableMock* resource = new ReleasableMock(released);

  SR_SAFE_RELEASE(resource);
  EXPECT_TRUE(released);
  EXPECT_EQ(resource, nullptr);

  SR_SAFE_RELEASE(resource);
  EXPECT_TRUE(released);
  EXPECT_EQ(resource, nullptr);
}

}  // namespace srlib::tests
