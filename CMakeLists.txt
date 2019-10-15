# wrenh-cone-lib: computes a Contact Wrench Cone using polyhedral presentation
# Copyright (C) 2019 Vincent Samy

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
