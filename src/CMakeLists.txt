declare_PID_Component(
    SHARED_LIB
    NAME wrench-cone-lib
    DIRECTORY WrenchConeLib
    CXX_STANDARD 14
)

declare_PID_Component_Dependency(
    COMPONENT wrench-cone-lib
    EXPORT EXTERNAL eigen
    INCLUDE_DIRS <eigen>/include
)

declare_PID_Component_Dependency(
    COMPONENT wrench-cone-lib
    EXPORT NATIVE eigen-cdd
    PACKAGE eigen-cdd
)

if(GEN_PYTHON_BINDINGS AND "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    declare_PID_Component(
        MODULE_LIB
        NAME pyWrenchConeLib
        DIRECTORY pyWrenchConeLib
        CXX_STANDARD 14
        INTERNAL COMPILER_OPTIONS -Wno-deprecated-declarations
    )

    declare_PID_Component_Dependency(
        COMPONENT pyWrenchConeLib
        EXPORT NATIVE wrench-cone-lib
        PACKAGE wrench-cone-lib
    )

    declare_PID_Component_Dependency(
        COMPONENT pyWrenchConeLib
        NATIVE eigen-cdd
        PACKAGE eigen-cdd
    )

    declare_PID_Component_Dependency(
        COMPONENT pyWrenchConeLib
        NATIVE cdd
        PACKAGE cdd
    )

    declare_PID_Component_Dependency(
        COMPONENT pyWrenchConeLib
        EXTERNAL boost-python
        PACKAGE boost
    )

    declare_PID_Component_Dependency(
        COMPONENT pyWrenchConeLib
        LINKS SHARED ${PYTHON_LIBRARIES}# ${Boost_LIBRARIES}
        INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}# ${Boost_INCLUDE_DIRS}
    )

    set_target_properties(pyWrenchConeLib${INSTALL_NAME_SUFFIX} PROPERTIES PREFIX "") # python needs a openphri.so file (without lib prefix)
    set_target_properties(pyWrenchConeLib${INSTALL_NAME_SUFFIX} PROPERTIES INSTALL_RPATH 
        "${${PROJECT_NAME}_INSTALL_PATH}/${${PROJECT_NAME}_INSTALL_LIB_PATH};${${PROJECT_NAME}_INSTALL_PATH}/${${PROJECT_NAME}_INSTALL_LIB_PATH}/../.rpath/pyWrenchConeLib${INSTALL_NAME_SUFFIX}") # set the rpath
    install(FILES pyWrenchConeLib/__init__.py DESTINATION ${WORKSPACE_DIR}/install/python2.7/WrenchConeLib)
    install(TARGETS pyWrenchConeLib${INSTALL_NAME_SUFFIX} DESTINATION ${WORKSPACE_DIR}/install/python2.7/WrenchConeLib)
endif()