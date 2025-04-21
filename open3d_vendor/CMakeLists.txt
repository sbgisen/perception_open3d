cmake_minimum_required(VERSION 3.8)

project(open3d_vendor)

find_package(ament_cmake REQUIRED)
find_package(Open3D QUIET)

macro(build_open3d)
  set(extra_cmake_args)

  include(ExternalProject)
  ExternalProject_Add(
    open3d-0.19.0
    GIT_REPOSITORY https://github.com/isl-org/Open3D.git
    GIT_TAG v0.19.0
    UPDATE_COMMAND ""
    TIMEOUT 300
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      -DCMAKE_INSTALL_INCLUDEDIR=include/${PROJECT_NAME}
      -DBUILD_SHARED_LIBS=ON
      -DBUILD_EXAMPLES=OFF
      -DBUILD_PYTHON_MODULE=OFF
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )
endmacro()

if(NOT Open3D_FOUND)
  build_open3d()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "open3d_vendor-extras.cmake"
)
