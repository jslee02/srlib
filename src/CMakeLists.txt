file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

set(osg_source ${CMAKE_CURRENT_SOURCE_DIR}/Renderer/SROpenSceneGraph.cpp)
if(NOT (SRLIB_ENABLE_OSG AND OPENSCENEGRAPH_FOUND))
  list(REMOVE_ITEM sources ${osg_source})
endif()

srlib_get_headers(headers)

add_library(${PROJECT_NAME} ${headers} ${sources})

if(SRLIB_ENABLE_OSG AND OPENSCENEGRAPH_FOUND)
  target_include_directories(${PROJECT_NAME} PRIVATE ${OPENSCENEGRAPH_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENSCENEGRAPH_LIBRARIES})
endif()
