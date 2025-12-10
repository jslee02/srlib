cmake_minimum_required(VERSION 3.16)

project(srlib VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SRLIB_ENABLE_OSG "Build OpenSceneGraph renderer (requires OpenSceneGraph)" OFF)
option(SRLIB_ENABLE_TESTS "Build srLib tests (requires GTest)" OFF)

#===============================================================================
# Find dependencies (target-based where possible)
#===============================================================================

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)  # Prefer GLVND OpenGL when available.
endif()
find_package(OpenGL QUIET)
if(NOT OPENGL_FOUND)
  message(FATAL_ERROR "OpenGL is required but was not found.")
endif()

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

# Collect dependency include dirs/libs for downstream targets.
set(SRLIB_THIRD_PARTY_LIBS "")
set(SRLIB_THIRD_PARTY_INCLUDE_DIRS "")

if(TARGET OpenGL::GL)
  list(APPEND SRLIB_THIRD_PARTY_LIBS OpenGL::GL)
elseif(OPENGL_LIBRARIES)
  list(APPEND SRLIB_THIRD_PARTY_LIBS ${OPENGL_LIBRARIES})
endif()
if(TARGET OpenGL::GLU)
  list(APPEND SRLIB_THIRD_PARTY_LIBS OpenGL::GLU)
elseif(OPENGL_glu_LIBRARY)
  list(APPEND SRLIB_THIRD_PARTY_LIBS ${OPENGL_glu_LIBRARY})
endif()
if(OPENGL_INCLUDE_DIR)
  list(APPEND SRLIB_THIRD_PARTY_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
endif()

if(TARGET GLEW::GLEW)
  list(APPEND SRLIB_THIRD_PARTY_LIBS GLEW::GLEW)
else()
  list(APPEND SRLIB_THIRD_PARTY_LIBS ${GLEW_LIBRARIES})
endif()
if(GLEW_INCLUDE_DIRS)
  list(APPEND SRLIB_THIRD_PARTY_INCLUDE_DIRS ${GLEW_INCLUDE_DIRS})
elseif(GLEW_INCLUDE_DIR)
  list(APPEND SRLIB_THIRD_PARTY_INCLUDE_DIRS ${GLEW_INCLUDE_DIR})
endif()

if(TARGET GLUT::GLUT)
  list(APPEND SRLIB_THIRD_PARTY_LIBS GLUT::GLUT)
else()
  list(APPEND SRLIB_THIRD_PARTY_LIBS ${GLUT_LIBRARIES})
endif()
if(GLUT_INCLUDE_DIR)
  list(APPEND SRLIB_THIRD_PARTY_INCLUDE_DIRS ${GLUT_INCLUDE_DIR})
endif()

# Optional OpenSceneGraph
set(_srlib_osg_components osg osgDB osgGA osgUtil osgViewer osgShadow)
if(SRLIB_ENABLE_OSG)
  find_package(OpenSceneGraph QUIET COMPONENTS ${_srlib_osg_components})
  if(OPENSCENEGRAPH_FOUND)
    set(SRLIB_OSG_LIBRARIES ${OPENSCENEGRAPH_LIBRARIES})
    set(SRLIB_OSG_INCLUDE_DIRS ${OPENSCENEGRAPH_INCLUDE_DIRS})
  else()
    message(STATUS "OpenSceneGraph not found; disabling OpenSceneGraph renderer (SROpenSceneGraph).")
    set(SRLIB_ENABLE_OSG OFF)
  endif()
endif()
if(NOT SRLIB_ENABLE_OSG)
  set(SRLIB_OSG_LIBRARIES "")
  set(SRLIB_OSG_INCLUDE_DIRS "")
endif()

add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(examples)

if(SRLIB_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
