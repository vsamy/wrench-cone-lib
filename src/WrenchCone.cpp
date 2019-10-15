// wrenh-cone-lib: computes a Contact Wrench Cone using polyhedral presentation
// Copyright (C) 2019 Vincent Samy

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "wcl/WrenchCone.h"

#include <Eigen/Geometry>
#include <cmath>

namespace wcl {

WrenchCone::WrenchCone(const Eigen::Vector3d& applicationPoint, const ContactSurface& cp)
    : ap_(applicationPoint)
    , cp_({ cp })
{
    resizeG();
}

WrenchCone::WrenchCone(const Eigen::Vector3d& applicationPoint, const std::vector<ContactSurface>& cps)
    : ap_(applicationPoint)
    , cp_(cps)
{
    resizeG();
}

Eigen::MatrixXd WrenchCone::getRays()
{
    computeG();
    return G_;
}

Eigen::MatrixXd WrenchCone::getHalfspaces()
{
    computeG();
    polyhedron_.setHrep(G_.transpose(), Eigen::VectorXd::Zero(G_.cols()));
    return polyhedron_.hrep().first;
}

/*
 * Private functions
 */

void WrenchCone::resizeG()
{
    Eigen::Index size(0);
    for (auto cp : cp_)
        size += cp.nrGenerators * cp.points.size();

    G_.resize(6, size);
}

void WrenchCone::computeG()
{
    Eigen::Index col(0);
    for (auto cp : cp_) {
        auto generators = generateCone(cp);
        for (auto p : cp.points) {
            Eigen::Vector3d r = cp.position + p - ap_;
            for (auto g : generators) {
#ifdef PLUCKER_NOTATION
                G_.col(col).segment<3>(0).noalias() = skewMatrix(r) * g;
                G_.col(col).segment<3>(3) = g;
#else
                G_.col(col).segment<3>(0) = g;
                G_.col(col).segment<3>(3).noalias() = skewMatrix(r) * g;
#endif
                col += 1;
            }
        }
    }
}

std::vector<Eigen::Vector3d> WrenchCone::generateCone(const ContactSurface& cp)
{
    std::vector<Eigen::Vector3d> generators(cp.nrGenerators);
    Eigen::Vector3d normal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d tan(Eigen::Vector3d::UnitX());
    double angle = std::atan(cp.mu);

    Eigen::Vector3d gen = Eigen::AngleAxisd(angle, tan) * normal;
    double step = (M_PI * 2.) / cp.nrGenerators;

    for (unsigned int i = 0; i < cp.nrGenerators; ++i) {
        generators[i] = cp.rotation.transpose() * Eigen::AngleAxisd(step * i, normal) * gen;
    }

    return generators;
}

Eigen::Matrix3d WrenchCone::skewMatrix(const Eigen::Vector3d v)
{
    Eigen::Matrix3d mat;
    mat << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return mat;
}

} // namespace wcl
