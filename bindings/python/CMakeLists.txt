# eigen-cddlib: wraps cddlib with Eigen matrix
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

configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/pyCopra/__init__.py COPYONLY)

find_package(pygen REQUIRED)
find_package(Python REQUIRED COMPONENTS Development)

if(${BUILD_TEST_STATIC_BOOST})
    set(Boost_USE_STATIC_LIBS ON)
    set(BUILD_SHARED_LIBS OFF)
    set(BOOST_DEFS "")
    set(BUILD_TYPE STATIC)
else()
    set(Boost_USE_STATIC_LIBS OFF)
    set(BUILD_SHARED_LIBS ON)
    set(BOOST_DEFS Boost::dynamic_linking)
    set(BUILD_TYPE SHARED)
endif()
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS python numpy)

add_library(pyWrenchConeLib ${BUILD_TYPE} pyWrenchConeLib.cpp)
target_link_libraries(pyWrenchConeLib PRIVATE ${PROJECT_NAME} pygen::pygen)

if(WIN32)
    set_target_properties(pyWrenchConeLib PROPERTIES SUFFIX ".pyd")
else()
    set_target_properties(pyWrenchConeLib PROPERTIES PREFIX "")
endif()

install(TARGETS pyWrenchConeLib DESTINATION "${PYTHON_INSTALL_PATH}/pyWrenchConeLib")
install(FILES __init__.py DESTINATION "${PYTHON_INSTALL_PATH}/__init__.py")

if(${BUILD_TESTING})
    add_subdirectory(tests)
endif()