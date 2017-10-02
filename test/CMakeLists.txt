
declare_PID_Component(
    TEST_APPLICATION
    NAME  TestWrenchConeLib
    DIRECTORY WrenchConeLib
)

declare_PID_Component_Dependency(
    COMPONENT TestWrenchConeLib
    NATIVE WrenchConeLib
)

declare_PID_Component_Dependency(
    COMPONENT TestWrenchConeLib
    EXTERNAL  boost-utest
    PACKAGE   boost
)

run_PID_Test(NAME test-TestWrenchConeLib COMPONENT TestWrenchConeLib)
add_test(NAME test-TestPyWrenchConeLib COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/pyWrenchConeLib/TestPyWrenchConeLib.py)