# minimum required version of cmake
cmake_minimum_required(VERSION 2.8.12)

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
find_package(OpenGL QUIET)

# GLEW
find_package(GLEW QUIET)

# GLUT
if(WIN32 AND NOT CYGWIN)
  set(GLUT_INCLUDE_DIR "@CMAKE_INSTALL_PREFIX@/include")
  set(GLUT_LIBRARIES glut32)
  set(HAVE_GLUT TRUE)
else()
  find_package(GLUT QUIET)
  if(GLUT_FOUND)
    set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
    set(HAVE_GLUT TRUE)
  else()
    set(HAVE_GLUT FALSE)
  endif()
endif()






include_directories(BEFORE include/srlib)

add_subdirectory(include)
add_subdirectory(src)

add_subdirectory(examples)
