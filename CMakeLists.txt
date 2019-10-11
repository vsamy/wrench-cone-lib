cmake_minimum_required(VERSION 3.12.4)

include(cmake/base.cmake)
include(cmake/msvc-specific.cmake)

set(PROJECT_NAME wcl)
set(PROJECT_DESCRIPTION "This library computes a Contact Wrench Cone using polyhedral presentation. ")
set(PROJECT_URL "https://github.com/vsamy/wrench-cone-lib")
set(PROJECT_DEBUG_POSTFIX "_d")
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "Generate and install standard headers")

set(DOXYGEN_USE_MATHJAX "YES")
set(CMAKE_CXX_STANDARD 11)

option(BUILD_TESTING "Disable unit tests." ON)
option(BUILD_EXAMPLES "Build examples." OFF)
option(BUILD_TEST_STATIC_BOOST "Build unit tests with static boost libraries" OFF)
option(PYTHON_BINDING "Generate python bindings." OFF)
option(PLUCKER_NOTATION "Plucker notation is used [ang lin], else the reverse is used [lin, ang]" OFF)

if(MSVC)
    set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif(MSVC)

setup_project()

add_project_dependency(eigen-cdd REQUIRED)

add_subdirectory(src)

if(NOT ${BUILD_TESTING})
    add_subdirectory(tests)
endif()

if(${PYTHON_BINDING})
    add_subdirectory(binding/python)
endif()

if(${BUILD_EXAMPLES})
    add_subdirectory(examples)
endif()

setup_project_finalize()
setup_project_package_finalize()
