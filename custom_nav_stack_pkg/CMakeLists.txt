cmake_minimum_required(VERSION 3.5)
project(custom_nav_stack_pkg)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) #To include markers visualization in rviz
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set(Boost_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
# set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
# set(BOOST_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
# set(BOOST_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
# set(Boost_INCLUDE_DIR /usr/include/)
# set(BOOST_INCLUDE_DIR /usr/include/)
# set(Boost_INCLUDE_DIRS /usr/include/)
# set(BOOST_INCLUDE_DIRS /usr/include/)
# find_package(Boost 1.55.0 ${QUIET_} COMPONENTS system filesystem date_time iostreams)
# find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem date_time iostreams) #Surprisingly this can find Boost but one present in PCL can't


# message( CMAKE_MODULE_PATH in mycmake:  ${CMAKE_MODULE_PATH} )
# message( Boost_FOUND in my cmakelist:  ${Boost_FOUND} )
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
if (${Python3_FOUND})
  include_directories(${Python3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

# https://pointclouds.org/documentation/tutorials/using_pcl_pcl_config.html
# When PCL is found, several related variables are set:
#     PCL_FOUND: set to 1 if PCL is found, otherwise unset
#     PCL_INCLUDE_DIRS: set to the paths to PCL installed headers and the dependency headers
#     PCL_LIBRARIES: set to the file names of the built and installed PCL libraries
#     PCL_LIBRARY_DIRS: set to the paths to where PCL libraries and 3rd party dependencies reside
#     PCL_VERSION: the version of the found PCL
#     PCL_COMPONENTS: lists all available components
#     PCL_DEFINITIONS: lists the needed preprocessor definitions and compiler flags

# https://ompl.kavrakilab.org/buildSystem.html
# CMake in your own project, you can simply use this command in your CMakeLists.txt: find_package(ompl). This will define the following variables:

#     OMPL_FOUND - TRUE
#     OMPL_INCLUDE_DIRS - The OMPL include directory
#     OMPL_LIBRARIES - The OMPL library
#     OMPLAPP_LIBRARIES - The OMPL.app libraries (if installed)
#     OMPL_VERSION - The OMPL version in the form <major>.<minor>.<patchlevel>
#     OMPL_MAJOR_VERSION - Major version
#     OMPL_MINOR_VERSION - Minor version
#     OMPL_PATCH_VERSION - Patch version



set(dependencies "rclcpp" "sensor_msgs" "geometry_msgs" "nav_msgs" "nav2_msgs" "lifecycle_msgs" "visualization_msgs" "PCL" "ompl" )#"libpcl-dev"


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(./include ${colcon_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# /usr/include/pcl-1.10 /usr/include/eigen3 

# Declare a cpp library
file(GLOB ompl_example_2d_sources
    src/ompl_example_2d.cpp
)

# file(GLOB ompl_example_2d_headers
#     include/ompl_example_2d/*.hpp
# )

add_library(${PROJECT_NAME} ${ompl_example_2d_sources}) #${ompl_example_2d_headers}
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Create Cpp executable
add_executable(lidar_cpp_executable src/lidar_cpp_node.cpp)
add_executable(occupancy_grid_generator src/occupancy_grid_generator.cpp)
add_executable(path_planner src/path_planner.cpp ${ompl_example_2d_sources}) #${ompl_example_2d_headers}
# add_executable(path_planner src/path_planner.cpp  src/ompl_example_2d.cpp include/ompl_example_2d/ompl_example_2d.hpp)
#ament_target_dependencies(lidar_cpp_executable rclcpp)
ament_target_dependencies(lidar_cpp_executable ${dependencies})
ament_target_dependencies(occupancy_grid_generator ${dependencies})
ament_target_dependencies(path_planner ${dependencies})
ament_export_dependencies(rclcpp)


# Install Cpp executables
install(TARGETS
  lidar_cpp_executable
  occupancy_grid_generator
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries (lidar_cpp_executable ${PCL_LIBRARIES})
target_link_libraries (occupancy_grid_generator  ${Python3_LIBRARIES})
target_link_libraries (path_planner  ${Python3_LIBRARIES} ${OMPL_LIBRARIES} )

# Install Python executables
install(PROGRAMS
  scripts/custom_nav_stack.py
  scripts/mode_control.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

