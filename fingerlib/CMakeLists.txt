cmake_minimum_required(VERSION 3.22)
project(fingerlib)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(Doxygen)

# Create library
add_library(fingerlib
    src/kinematics.cpp
    src/jacobian_lookup.cpp
)

# Utility executable for Jacobian precomputation
add_executable(precompute_jacobian
    src/precompute_jacobian.cpp
)
target_link_libraries(precompute_jacobian PRIVATE fingerlib)
target_compile_features(precompute_jacobian PRIVATE cxx_std_23)

# Simple tension demo executable (build for local testing without firmware)
add_executable(tension_demo
    src/tension_demo.cpp
)
target_link_libraries(tension_demo PRIVATE fingerlib)
target_compile_features(tension_demo PRIVATE cxx_std_23)

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

include(CTest)

target_link_libraries(fingerlib
    PUBLIC
    Eigen3::Eigen
)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find Catch2 for unit testing
    find_package(Catch2 3 REQUIRED)

    # Add test executables
    add_executable(fingerlib_tests
        tests/kinematics_tests.cpp
        tests/jacobian_lookup_tests.cpp
        tests/nnls_tests.cpp
    ) 
    target_link_libraries(fingerlib_tests PRIVATE Catch2::Catch2WithMain fingerlib)
    target_include_directories(fingerlib_tests PRIVATE include)

    # Set C++23 standard for tests
    target_compile_features(fingerlib_tests PRIVATE cxx_std_23)

    # Register tests with CTest
    add_test(NAME fingerlib_tests COMMAND fingerlib_tests)
endif()

# Use target_include_directories so that #include"mylibrary/header.hpp" works
target_include_directories(fingerlib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# Specify additional compilation flags for the library
target_compile_options(fingerlib PUBLIC -Wall -Wextra -pedantic)

# Enable c++23 support
target_compile_features(fingerlib PUBLIC cxx_std_23)

# Install fingerlib
install(TARGETS fingerlib
    EXPORT fingerlib-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS precompute_jacobian
    RUNTIME DESTINATION bin
)

install(TARGETS tension_demo
    RUNTIME DESTINATION bin
)

install(FILES jacobian_transposes.csv
    DESTINATION share/${PROJECT_NAME}/config
)

install(EXPORT fingerlib-targets
    FILE fingerlib-config.cmake
    NAMESPACE fingerlib::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
