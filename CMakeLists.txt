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
find_package(GLUT QUIET)
if(NOT GLUT_FOUND AND DEFINED ENV{CONDA_PREFIX})
  # Fallback for conda/pixi layouts on all platforms.
  set(_conda $ENV{CONDA_PREFIX})
  find_path(GLUT_INCLUDE_DIR
    NAMES glut.h
    HINTS
      "${_conda}/include"
      "${_conda}/Library/include"
  )
  find_library(GLUT_glut_LIBRARY
    NAMES glut freeglut freeglut_static
    HINTS
      "${_conda}/lib"
      "${_conda}/Library/lib"
  )
  if(GLUT_INCLUDE_DIR AND GLUT_glut_LIBRARY)
    set(GLUT_FOUND TRUE)
  endif()
endif()

if(NOT GLUT_FOUND)
  message(FATAL_ERROR "GLUT is required but was not found. Install freeglut or provide GLUT_ROOT.")
endif()

set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
set(HAVE_GLUT TRUE)






include_directories(BEFORE include/srlib)

add_subdirectory(include)
add_subdirectory(src)

add_subdirectory(examples)
