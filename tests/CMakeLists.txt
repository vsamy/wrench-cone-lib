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

enable_testing()

if(${BUILD_TEST_STATIC_BOOST})
    set(Boost_USE_STATIC_LIBS ON)
    set(BUILD_SHARED_LIBS OFF)
    set(BOOST_DEFS "")
else()
    set(Boost_USE_STATIC_LIBS OFF)
    set(BUILD_SHARED_LIBS ON)
    set(BOOST_DEFS Boost::dynamic_linking)
endif()
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(wclTest TestWrenchConeLib.cpp)
target_link_libraries(wclTest PRIVATE Boost::unit_test_framework Boost::disable_autolinking ${BOOST_DEFS} ${PROJECT_NAME})
add_test(wclTestUnit wclTest)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(wclTest)

