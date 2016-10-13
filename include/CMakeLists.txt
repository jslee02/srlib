file(GLOB_RECURSE
  headers
  ${CMAKE_CURRENT_SOURCE_DIR}/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
)

srlib_add_headers(${headers})
