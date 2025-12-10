set(example_name "MobileRobots")

file(GLOB headers "*.h")
file(GLOB sources "*.cpp")

add_executable(${example_name} ${headers} ${sources})

target_link_libraries(${example_name} PRIVATE ${PROJECT_NAME})

set_target_properties(
  ${example_name} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/examples"
)
