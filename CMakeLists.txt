cmake_minimum_required(VERSION 3.8)
project(custom_turtlesim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Arquivos de serviço
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Spawn.srv"
  "srv/SetBgColor.srv"
  "srv/ChangeSprite.srv"
  "srv/Kill.srv"
  "srv/GetTurtlePosition.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Instalar arquivos Python
install(
  DIRECTORY custom_turtlesim/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Instalar scripts executáveis
install(
  PROGRAMS
    custom_turtlesim/simulator_node.py
    custom_turtlesim/teleop_node.py
    custom_turtlesim/atividade_3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY custom_turtlesim/resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

ament_package()
