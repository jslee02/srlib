# minimum required version of cmake
cmake_minimum_required(VERSION 2.8.12)

# Project name. Stored in ${PROJECT_NAME}.
project(srlib)

include_directories(BEFORE include/srlib)

add_subdirectory(include)
add_subdirectory(src)
