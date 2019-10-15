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

set(SOURCES
    ContactSurface.cpp
    WrenchCone.cpp
)

set(HEADERS
    ../include/wcl/api.h
    ../include/wcl/ContactSurface.h
    ../include/wcl/WrenchCone.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-Dwcl_EXPORTS")
if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
endif()
if(${PLUCKER_NOTATION})
    target_compile_definitions(${PROJECT_NAME} PUBLIC PLUCKER_NOTATION)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC eigen-cdd::eigen-cdd)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}") 

install(TARGETS ${PROJECT_NAME}
    EXPORT "${TARGETS_EXPORT_NAME}"
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
