


declare_PID_Component(TEST_APPLICATION NAME test-wrench-cone-lib DIRECTORY wrench_cone_lib)
declare_PID_Component_Dependency(COMPONENT test-wrench-cone-lib NATIVE wrench-cone-lib)
declare_PID_Component_Dependency(COMPONENT test-wrench-cone-lib EXTERNAL  boost-utest PACKAGE boost)

run_PID_Test(NAME testing-wrench-cone-lib COMPONENT test-wrench-cone-lib)
run_PID_Test(NAME testing-python-wrapper PYTHON ARGUMENTS pyWrenchConeLib/TestPyWrenchConeLib.py)
