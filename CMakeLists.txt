# minimum required version of cmake
cmake_minimum_required(VERSION 3.11)

# Project name. Stored in ${PROJECT_NAME}.
project(srlib)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(srlibUtils)

#===============================================================================
# Find dependencies
#===============================================================================

#-------------------------------------------------------------------------------
# Optional dependencies
#-------------------------------------------------------------------------------

# OpenGL
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)  # Prefer GLVND OpenGL when available.
endif()
find_package(OpenGL QUIET)

# GLEW
find_package(GLEW QUIET)

# GLUT
find_package(GLUT REQUIRED)
set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
set(HAVE_GLUT TRUE)






include_directories(BEFORE include/srlib)

add_subdirectory(include)
add_subdirectory(src)

add_subdirectory(examples)
