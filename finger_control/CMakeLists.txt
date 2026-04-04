cmake_minimum_required(VERSION 3.20)
project(finger_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fingerlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Doxygen)

include(CTest)

# Add fingerlib as library
add_library(fingerlib STATIC
    ../fingerlib/src/kinematics.cpp
)
target_include_directories(fingerlib PUBLIC ../fingerlib/include)

# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Use README.md as homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # Documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

# Add dependencies to nodes
add_executable(finger_pd_control src/finger_pd_control.cpp)
target_link_libraries(finger_pd_control PUBLIC
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    Eigen3::Eigen
    rclcpp::rclcpp
    fingerlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ROS2 tests
  # find_package(catch_ros2 REQUIRED)
endif()

# Force c++23 in all exec + libs
set_target_properties(finger_pd_control PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED YES
)
set_target_properties(fingerlib PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED YES
)

install(PROGRAMS
  scripts/finger_target_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS finger_pd_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
