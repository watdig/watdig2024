find_package(rosidl_default_generators REQUIRED)

set(msg_files
"msg/position.msg"
)

set(srv_files
"srv/Position.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)