set(example_name "Template")

file(GLOB headers "*.h")
file(GLOB sources "*.cpp")

add_executable(${example_name} ${headers} ${sources})

target_link_libraries(
  ${example_name}
  ${PROJECT_NAME}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLUT_LIBRARIES}
)

set_target_properties(
  ${example_name} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/examples"
)
