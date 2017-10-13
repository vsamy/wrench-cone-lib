declare_PID_Component(
    SHARED_LIB 
    NAME wrench-cone-lib 
    DIRECTORY wrench_cone_lib
    CXX_STANDARD 14
)

declare_PID_Component_Dependency(
    COMPONENT wrench-cone-lib
    EXPORT EXTERNAL eigen 
    PACKAGE eigen
)

declare_PID_Component_Dependency(
    COMPONENT wrench-cone-lib
    EXPORT NATIVE eigen-cdd 
    PACKAGE eigen-cdd
)

#declare the python module

declare_PID_Component(
    MODULE_LIB
    NAME WrenchConeLib
    DIRECTORY pyWrenchConeLib
    CXX_STANDARD 14
    INTERNAL COMPILER_OPTIONS -Wno-deprecated-declarations
)

declare_PID_Component_Dependency(
    COMPONENT WrenchConeLib 
    NATIVE wrench-cone-lib
)

declare_PID_Component_Dependency(
    COMPONENT WrenchConeLib
    EXPORT NATIVE pygen
    PACKAGE pygen-converter
)

get_PID_Platform_Info(PYTHON PY_VERSION)

if(PY_VERSION VERSION_LESS 3.0)
    #using python2 to manage python wrappers
    declare_PID_Component_Dependency(
        COMPONENT WrenchConeLib 
        EXTERNAL boost-python 
        PACKAGE boost
    )
else()
    #using python3 to manage python wrappers
    declare_PID_Component_Dependency(
        COMPONENT WrenchConeLib 
        EXTERNAL boost-python3 
        PACKAGE boost
    )
endif()
