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
if(NOT GLEW_FOUND)
  set(_glew_hints)
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _glew_hints
      "$ENV{CONDA_PREFIX}/include"
      "$ENV{CONDA_PREFIX}/lib"
      "$ENV{CONDA_PREFIX}/Library/include"
      "$ENV{CONDA_PREFIX}/Library/lib"
    )
  endif()
  if(DEFINED ENV{HOMEBREW_PREFIX})
    list(APPEND _glew_hints
      "$ENV{HOMEBREW_PREFIX}/include"
      "$ENV{HOMEBREW_PREFIX}/lib"
    )
  endif()
  if(EXISTS "/opt/homebrew")
    list(APPEND _glew_hints "/opt/homebrew/include" "/opt/homebrew/lib")
  endif()
  if(EXISTS "/usr/local")
    list(APPEND _glew_hints "/usr/local/include" "/usr/local/lib")
  endif()

  find_path(GLEW_INCLUDE_DIR
    NAMES GL/glew.h
    HINTS ${_glew_hints}
  )
  find_library(GLEW_LIBRARY
    NAMES GLEW glew32 glew32s
    HINTS ${_glew_hints}
  )
  if(GLEW_INCLUDE_DIR AND GLEW_LIBRARY)
    set(GLEW_FOUND TRUE)
    set(GLEW_LIBRARIES ${GLEW_LIBRARY})
    set(GLEW_INCLUDE_DIRS ${GLEW_INCLUDE_DIR})
  endif()
endif()

# GLUT
find_package(GLUT QUIET)
if(NOT GLUT_FOUND)
  # Fallback search for common package manager prefixes (conda/pixi, Homebrew).
  set(_glut_hints)
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _glut_hints
      "$ENV{CONDA_PREFIX}/include"
      "$ENV{CONDA_PREFIX}/lib"
      "$ENV{CONDA_PREFIX}/Library/include"
      "$ENV{CONDA_PREFIX}/Library/lib"
    )
  endif()
  if(DEFINED ENV{HOMEBREW_PREFIX})
    list(APPEND _glut_hints
      "$ENV{HOMEBREW_PREFIX}/include"
      "$ENV{HOMEBREW_PREFIX}/lib"
    )
  endif()
  if(EXISTS "/opt/homebrew")
    list(APPEND _glut_hints "/opt/homebrew/include" "/opt/homebrew/lib")
  endif()
  if(EXISTS "/usr/local")
    list(APPEND _glut_hints "/usr/local/include" "/usr/local/lib")
  endif()

  find_path(GLUT_INCLUDE_DIR
    NAMES glut.h
    HINTS ${_glut_hints}
  )
  find_library(GLUT_glut_LIBRARY
    NAMES glut freeglut freeglut_static
    HINTS ${_glut_hints}
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

# Add graphics include paths early so headers like GL/glew.h, GL/glut.h are found
include_directories(BEFORE ${GLEW_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR})






include_directories(BEFORE include/srlib)

add_subdirectory(include)
add_subdirectory(src)

add_subdirectory(examples)
