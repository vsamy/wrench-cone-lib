#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TestWrenchConeLib

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/test/unit_test.hpp>
#include <chrono>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <utility>

#include <WrenchConeLib/CWC.h>
#include <WrenchConeLib/ContactSurface.h>

BOOST_AUTO_TEST_CASE(HUMANOID_FEET_RAYS)
{
    using namespace std::chrono;

    std::vector<std::size_t> timings;

    for (int i = 0; i < 100; ++i) {
        auto initTimer = high_resolution_clock::now();

        // Positions
        Eigen::Vector3d com(Eigen::Vector3d::Zero());
        Eigen::Vector3d lf(0.2, 0.15, 0.1);
        Eigen::Vector3d rf(-0.4, 0, 0);

        // Orientations
        Eigen::Matrix3d E_0_lf = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d E_0_rf = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();

        // Contact info
        double mu = 0.5;
        int nrg = 4;

        // Contact surface
        auto cLf = wcl::rectangularSurface(0.11, 0.05, lf, E_0_lf, mu, nrg);
        auto cRf = wcl::rectangularSurface(0.11, 0.05, rf, E_0_rf, mu, nrg);

        wcl::CWC cwc(com, { cLf, cRf });
        auto spanMat = cwc.getRays();

        timings.push_back(duration_cast<microseconds>(high_resolution_clock::now() - initTimer).count());
    }

    std::cout << "Worst of 100: " << *std::max_element(timings.begin(), timings.end()) << "us" << std::endl;
    std::cout << "Best of 100: " << *std::min_element(timings.begin(), timings.end()) << "us" << std::endl;
}

BOOST_AUTO_TEST_CASE(HUMANOID_FEET_HALFSPACES)
{
    using namespace std::chrono;

    std::vector<std::size_t> timings;

    for (int i = 0; i < 100; ++i) {
        auto initTimer = high_resolution_clock::now();

        // Positions
        Eigen::Vector3d com(Eigen::Vector3d::Zero());
        Eigen::Vector3d lf(0.2, 0.15, 0.1);
        Eigen::Vector3d rf(-0.4, 0, 0);

        // Orientations
        Eigen::Matrix3d E_0_lf = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d E_0_rf = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();

        // Contact info
        double mu = 0.5;
        int nrg = 4;

        // Contact surface
        auto cLf = wcl::rectangularSurface(0.11, 0.05, lf, E_0_lf, mu, nrg);
        auto cRf = wcl::rectangularSurface(0.11, 0.05, rf, E_0_rf, mu, nrg);

        wcl::CWC cwc(com, { cLf, cRf });
        auto spanMat = cwc.getHalfSpaces();

        timings.push_back(duration_cast<microseconds>(high_resolution_clock::now() - initTimer).count());
    }

    std::cout << "Worst of 100: " << *std::max_element(timings.begin(), timings.end()) << "us" << std::endl;
    std::cout << "Best of 100: " << *std::min_element(timings.begin(), timings.end()) << "us" << std::endl;
}