cmake_minimum_required(VERSION 3.5)
project(open3d_conversions)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# system dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

ament_auto_add_library(open3d_conversions src/open3d_conversions.cpp)
target_link_libraries(open3d_conversions ${Open3D_LIBRARIES})
target_compile_definitions(open3d_conversions PUBLIC "OPEN3D_CONVERSIONS_BUILDING_LIBRARY")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  target_link_libraries(
    test_open3d_conversions
    open3d_conversions
    ${Open3D_LIBRARIES}
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
