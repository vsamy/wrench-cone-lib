declare_PID_Component(
    EXAMPLE_APPLICATION
    NAME cppPerf
    DIRECTORY WrenchConeLibPerf
    CXX_STANDARD 14
)

declare_PID_Component_Dependency(
    COMPONENT cppPerf
    NATIVE WrenchConeLib
    PACKAGE wrench-cone-lib
)


get_PID_Platform_Info(PYTHON PY_VERSION)
if( PY_VERSION #python is configured in the workspace
    AND CMAKE_BUILD_TYPE STREQUAL "Release")
    install(FILES pyWrenchConeLibPerf/example.py
            DESTINATION "${${PROJECT_NAME}_INSTALL_PATH}/${${PROJECT_NAME}_INSTALL_BIN_PATH}"
            RENAME "pyPerf.py"
            PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endif()
