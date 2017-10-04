/*      File: CWC.cpp
*       This file is part of the program wrench-cone-lib
*       Program description : This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. Python bindings are also available.
*       Copyright (C) 2017 -  Vincent Samy (LIRMM). All Right reserved.
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
#include <wcl/CWC.h>

#include <Eigen/Geometry>
#include <cmath>

namespace wcl {

CWC::CWC(const Eigen::Vector3d& com, const ContactSurface& cp)
    : com_(com)
    , cp_({ cp })
{
    resizeG();
}

CWC::CWC(const Eigen::Vector3d& com, const std::vector<ContactSurface>& cps)
    : com_(com)
    , cp_(cps)
{
    resizeG();
}

/* v-rep
     *
     */
Eigen::MatrixXd CWC::getRays()
{
    computeG();
    return G_;
}

/* h-rep
     *
     */
Eigen::MatrixXd CWC::getHalfSpaces()
{
    computeG();
    polyhedron_.hrep(G_.transpose(), Eigen::VectorXd::Zero(G_.cols()));
    return polyhedron_.hrep().first;
}

/*
 * Private functions
 */

void CWC::resizeG()
{
    Eigen::Index size(0);
    for (auto cp : cp_)
        size += cp.nrGenerators * cp.points.size();

    G_.resize(6, size);
}

void CWC::computeG()
{
    Eigen::Index col(0);
    for (auto cp : cp_) {
        auto generators = generateCone(cp);
        for (auto p : cp.points) {
            Eigen::Vector3d r = cp.r_0_s + p - com_;
            for (auto g : generators) {
                G_.col(col).segment<3>(0) = g;
                G_.col(col).segment<3>(3).noalias() = skewMatrix(r) * g;
                col += 1;
            }
        }
    }
}

std::vector<Eigen::Vector3d> CWC::generateCone(const ContactSurface& cp)
{
    std::vector<Eigen::Vector3d> generators(cp.nrGenerators);
    Eigen::Vector3d normal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d tan(Eigen::Vector3d::UnitX());
    double angle = std::atan(cp.mu);

    Eigen::Vector3d gen = Eigen::AngleAxisd(angle, tan) * normal;
    double step = (M_PI * 2.) / cp.nrGenerators;

    for (int i = 0; i < cp.nrGenerators; ++i) {
        generators[i] = cp.E_0_s.transpose() * Eigen::AngleAxisd(step * i, normal) * gen;
    }

    return generators;
}

Eigen::Matrix3d CWC::skewMatrix(const Eigen::Vector3d v)
{
    Eigen::Matrix3d mat;
    mat << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return mat;
}

} // namespace wcl
