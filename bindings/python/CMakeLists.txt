configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/pyCopra/__init__.py COPYONLY)

find_package(pygen-converter REQUIRED)

if(NOT PYTHON_INSTALL_PATH)
    set(PYTHON_INSTALL_PATH ${PYTHON_SITELIB})
endif()

add_library(pyWrenchConeLib SHARED pyWrenchConeLib.cpp)
target_link_libraries(pyWrenchConeLib PUBLIC ${PROJECT_NAME} pygen-converter::pygen-converter)

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