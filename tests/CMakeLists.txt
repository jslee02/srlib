cmake_minimum_required(VERSION 3.16)

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(srlib_smoke_test
  srlib_smoke_test.cpp
)

target_link_libraries(srlib_smoke_test
  PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
)

gtest_discover_tests(srlib_smoke_test)
