/*      File: WrenchCone.cpp
*       This file is part of the program WrenchConeLib
*       Program description : This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. Python bindings are also available.
*       Copyright (C) 2017 -  vsamy (LIRMM). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the CeCILL-C license as published by
*       the CEA CNRS INRIA, either version 1
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       CeCILL-C License for more details.
*
*       You should have received a copy of the CeCILL-C License
*       along with this software. If not, it can be found on the official website
*       of the CeCILL licenses family (http://www.cecill.info/index.en.html).
*/
#include <wcl/WrenchCone.h>

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
    polyhedron_.hrep(G_.transpose(), Eigen::VectorXd::Zero(G_.cols()));
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

    for (int i = 0; i < cp.nrGenerators; ++i) {
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
