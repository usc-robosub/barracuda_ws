cmake_minimum_required(VERSION 3.8)
project(barracuda_ping_360)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install python package so import works
ament_python_install_package(${PROJECT_NAME})

# install scripts (small wrappers / executables)
install(PROGRAMS
  scripts/ping360_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and config
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME}/)
ament_package()