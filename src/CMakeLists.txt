file(GLOB_RECURSE SRLIB_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

set(_osg_source ${CMAKE_CURRENT_SOURCE_DIR}/Renderer/SROpenSceneGraph.cpp)
if(NOT SRLIB_ENABLE_OSG)
  list(REMOVE_ITEM SRLIB_SOURCES ${_osg_source})
endif()

add_library(${PROJECT_NAME} ${SRLIB_SOURCES} ${SRLIB_HEADERS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/srlib>
    $<INSTALL_INTERFACE:include/srlib>
    ${SRLIB_THIRD_PARTY_INCLUDE_DIRS}
    $<$<BOOL:${SRLIB_ENABLE_OSG}>:${SRLIB_OSG_INCLUDE_DIRS}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${SRLIB_THIRD_PARTY_LIBS}
  PRIVATE
    $<$<BOOL:${SRLIB_ENABLE_OSG}>:${SRLIB_OSG_LIBRARIES}>
)

if(MSVC)
  # Avoid std::byte vs Windows byte conflict in SDK headers.
  target_compile_definitions(${PROJECT_NAME} PUBLIC _HAS_STD_BYTE=0)
endif()
