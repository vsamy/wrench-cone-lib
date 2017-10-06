declare_PID_Component(
    EXAMPLE_APPLICATION
    NAME cppPerf
    DIRECTORY WrenchConeLibPerf
    CXX_STANDARD 14
)

declare_PID_Component_Dependency(
    COMPONENT cppPerf
    NATIVE wrench-cone-lib
    PACKAGE wrench-cone-lib
)

declare_PID_Component_Dependency(
    COMPONENT cppPerf
    EXTERNAL eigen 
    PACKAGE eigen
)