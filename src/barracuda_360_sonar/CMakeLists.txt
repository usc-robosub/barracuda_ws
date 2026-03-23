cmake_minimum_required(VERSION 3.8)
project(barracuda_360_sonar)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python module directory so it can be imported as:
# from ping360_node import ...
ament_python_install_package(ping360_node)

# Install the runnable entry script.
# This makes the installed executable name match the launch file:
# executable='ping360_node'
install(PROGRAMS
  scripts/ping360_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ping360_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files if the directory exists
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()