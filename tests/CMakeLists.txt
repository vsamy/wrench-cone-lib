enable_testing()

if(${BUILD_TEST_STATIC_BOOST})
    set(Boost_USE_STATIC_LIBS ON)
    set(BUILD_SHARED_LIBS OFF)
    set(BOOST_DEFS "")
else()
    set(Boost_USE_STATIC_LIBS OFF)
    set(BUILD_SHARED_LIBS ON)
    set(BOOST_DEFS Boost::dynamic_linking)
endif()
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(wclTest TestWrenchConeLib.cpp)
target_link_libraries(wclTest PRIVATE Boost::unit_test_framework Boost::disable_autolinking ${BOOST_DEFS} ${PROJECT_NAME})
add_test(wclTestUnit wclTest)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(wclTest)

