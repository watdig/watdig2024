find_package(rosidl_default_generators REQUIRED)

set(msg_files
"msg/position.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)