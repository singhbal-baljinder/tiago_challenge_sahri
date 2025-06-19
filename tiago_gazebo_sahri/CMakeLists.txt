cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(tiago_gazebo_sahri)


#
find_package(tiago_simulation REQUIRED)
find_package(pal_gazebo_worlds REQUIRED)

# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/tiago_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
