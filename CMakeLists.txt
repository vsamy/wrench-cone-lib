cmake_minimum_required(VERSION 3.1.3)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/msvc-specific.cmake)

set(PROJECT_NAME wrench-cone-lib)
set(PROJECT_DESCRIPTION "This library computes a Contact Wrench Cone using polyhedral presentation. ")
set(PROJECT_URL "https://github.com/vsamy/wrench-cone-lib")
set(PROJECT_DEBUG_POSTFIX "_d")

set(INSTALL_GENERATED_HEADERS OFF)
set(DOXYGEN_USE_MATHJAX "YES")

option(BUILD_TESTING "Disable unit tests." ON)
option(PYTHON_BINDING "Generate python bindings." OFF)
option(PLUCKER_NOTATION "Plucker notation is used [ang lin], else the reverse is used [lin, ang]" OFF)

if(MSVC)
    set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif(MSVC)

project(${PROJECT_NAME})
setup_project()

set(BOOST_REQUIRED 1.48)
set(Eigen_REQUIRED "eigen3 >= 3.2.0")

add_project_dependency("Eigen3 eigen-cdd" REQUIRED)

add_subdirectory(src)

if(NOT ${BUILD_TESTING})
  add_subdirectory(tests)
endif()

if(${PYTHON_BINDING})
    add_subdirectory(binding/python)
endif()

setup_project_finalize()
setup_project_package_finalize()
