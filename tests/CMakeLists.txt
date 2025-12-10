cmake_minimum_required(VERSION 3.16)

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

set(SRLIB_TEST_SOURCES
  srlib_smoke_test.cpp
  common/common_types_test.cpp
  common/common_utils_test.cpp
)

add_executable(srlib_tests ${SRLIB_TEST_SOURCES})

target_link_libraries(srlib_tests
  PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
)

gtest_discover_tests(srlib_tests)
