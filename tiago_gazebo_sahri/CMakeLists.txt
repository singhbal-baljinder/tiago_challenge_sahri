cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(tiago_gazebo_sahri)

# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(pal_gazebo_worlds REQUIRED)

# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/tiago_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

#get_filename_component(PGW_PREFIX
  #"${pal_gazebo_worlds_DIR}/../../.." ABSOLUTE)

#file(GLOB SAHRI_WORLDS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.world")

#if(SAHRI_WORLDS)
  #install(FILES ${SAHRI_WORLDS}
          #DESTINATION "${PGW_PREFIX}/share/pal_gazebo_worlds/worlds")
#endif()

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
