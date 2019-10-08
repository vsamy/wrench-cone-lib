/*      File: ContactSurface.h
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
#pragma once

#include "wcl/api.h"
#include <Eigen/Core>
#include <vector>

namespace wcl {

/** Representation of a contact surface
 * It handles all the needed information to compute the CWC.
 */
struct WCL_DLLAPI ContactSurface {
    Eigen::Vector3d position; /**< Position vector of the surface in the world coordinates */
    Eigen::Matrix3d rotation; /**< Rotation matrix of the surface from the world frame to the surface frame */
    std::vector<Eigen::Vector3d> points; /**< List of points belonging to the surface in the surface coordinates */
    double mu = 0.7; /**< Static friction coefficient of the surface */
    unsigned int nrGenerators = 4; /**< Number of generators to approximate the friction cone */
};

/** Function that build a rectangular surface. The z-axis of the surface is its normal.
 * The x-axis, y-axis and z-axis are defined with regard to the given rotation matrix E_0_s.
 * \param xHalfLength the half-length of the surface on the x-axis
 * \param yHalfLength the half-length of the surface on the y-axis
 * \param position position vector of the surface in the world coordinates
 * \param rotation rotation matrix of the surface from the world frame to the surface frame
 * \param mu static friction coefficient of the surface
 * \param nrGenerators number of generators to approximate the friction cone
 * \return A squared ContactSurface
 */
WCL_DLLAPI ContactSurface rectangularSurface(double xHalfLength, double yHalfLength, const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotation, double mu = 0.7, unsigned int nrGenerators = 4);

} // namespace wcl
