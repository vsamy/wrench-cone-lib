enable_testing()

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(wclTest TestWrenchConeLib.cpp)
target_link_libraries(wclTest PRIVATE ${PROJECT_NAME} Boost::unit_test_framework)
add_test(wclTestUnit wclTest)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(wclTest)

