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
if(${PLUCKER_NOTATION})
    target_compile_definitions(${PROJECT_NAME} PUBLIC PLUCKER_NOTATION)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC eigen-cdd::eigen-cdd pygen::pygen)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}") 
