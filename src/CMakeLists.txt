file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

srlib_get_headers(headers)

add_library(${PROJECT_NAME} ${headers} ${sources})
