
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wcl/ContactSurface.h>
#include <wcl/WrenchCone.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>

static int NUMBER_OF_TESTS = 100;

// Left foot:
// - pos = array([ 0.2 ,  0.15,  0.1 ])
// - rpy = array([ 0.4, -0. ,  0. ])
// - half-length = 0.11
// - half-width = 0.05
// - friction = 0.5

// Right foot:
// - pos = array([-0.2  , -0.195,  0.   ])
// - rpy = array([-0.4,  0. ,  0. ])
// - half-length = 0.11
// - half-width = 0.05
// - friction = 0.5

// Contact Wrench Cone at [0.0, 0.0, 0.0]:

struct System {
    // Positions
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf = Eigen::Vector3d(0.2, 0.15, 0.1);
    Eigen::Vector3d rf = Eigen::Vector3d(-0.4, 0, 0);

    // Orientations
    Eigen::Matrix3d E_0_lf = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d E_0_rf = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // Surface size
    double halfLength = 0.11;
    double halfWidth = 0.05;

    // Contact info
    double mu = 0.5;
    int nrg = 4;
} sys;

void computeRaysPerf()
{
    using namespace std::chrono;

    std::vector<std::size_t> timings;

    for (int i = 0; i < NUMBER_OF_TESTS; ++i) {
        auto initTimer = high_resolution_clock::now();

        // Contact surface
        auto cLf = wcl::rectangularSurface(sys.halfLength, sys.halfWidth, sys.lf, sys.E_0_lf, sys.mu, sys.nrg);
        auto cRf = wcl::rectangularSurface(sys.halfLength, sys.halfWidth, sys.rf, sys.E_0_rf, sys.mu, sys.nrg);

        wcl::WrenchCone wrenchCone(sys.com, { cLf, cRf });
        auto spanMat = wrenchCone.getRays();

        timings.push_back(duration_cast<microseconds>(high_resolution_clock::now() - initTimer).count());
    }

    std::cout << "Rays computation time" << std::endl;
    auto res = std::minmax_element(timings.begin(), timings.end());
    std::cout << "Worst of " << NUMBER_OF_TESTS << ": " << *res.second << "µs" << std::endl;
    std::cout << "Best of " << NUMBER_OF_TESTS << ": " << *res.first << "µs" << std::endl;
    auto acc = std::accumulate(timings.begin(), timings.end(), size_t(0));
    std::cout << "Mean of " << NUMBER_OF_TESTS << ": " << acc / NUMBER_OF_TESTS << "µs" << std::endl;
}

void computeHalfspacesPerf()
{
    using namespace std::chrono;

    std::vector<std::size_t> timings;

    for (int i = 0; i < NUMBER_OF_TESTS; ++i) {
        auto initTimer = high_resolution_clock::now();

        // Contact surface
        auto cLf = wcl::rectangularSurface(sys.halfLength, sys.halfWidth, sys.lf, sys.E_0_lf, sys.mu, sys.nrg);
        auto cRf = wcl::rectangularSurface(sys.halfLength, sys.halfWidth, sys.rf, sys.E_0_rf, sys.mu, sys.nrg);

        wcl::WrenchCone wrenchCone(sys.com, { cLf, cRf });
        auto spanMat = wrenchCone.getHalfspaces();

        timings.push_back(duration_cast<microseconds>(high_resolution_clock::now() - initTimer).count());
    }

    auto res = std::minmax_element(timings.begin(), timings.end());
    auto acc = std::accumulate(timings.begin(), timings.end(), size_t(0));
    std::cout << std::endl << "Halfspaces computation time" << std::endl;
    std::cout << "Worst of " << NUMBER_OF_TESTS << ": " << *res.second << "µs" << std::endl;
    std::cout << "Best of " << NUMBER_OF_TESTS << ": " << *res.first << "µs" << std::endl;
    std::cout << "Mean of " << NUMBER_OF_TESTS << ": " << acc / NUMBER_OF_TESTS << "µs" << std::endl;
}

int main()
{
    computeRaysPerf();
    computeHalfspacesPerf();

    return EXIT_SUCCESS;
}