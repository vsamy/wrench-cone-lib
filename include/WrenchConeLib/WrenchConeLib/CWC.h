/*      File: CWC.h
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
#pragma once

#include "ContactSurface.h"
#include <eigen-cdd/Polyhedron.h>
#include <vector>

namespace wcl {

/** CWC (Contact Wrench Cone) is the main class of this lib.
 * It computes the Contact Wrench Cone using 6D polyhedrons.
 */
class CWC {
public:
    /** Constructor
     * \param com The center of mass of the system
     * \param cp A contact surface
     */
    CWC(const Eigen::Vector3d& com, const ContactSurface& cp);
    /** Constructor
     * \param com The center of mass of the system
     * \param cps A list of contact surfaces
     */
    CWC(const Eigen::Vector3d& com, const std::vector<ContactSurface>& cps);


    /** Retrieve the V-representation (vertexes and rays of the polyhedron) of the CWC
     * \return A matrix \f$G\f$ such that \f$G\lambda = w_g, \lambda \geq 0\f$ with \f$w_g\f$ the contact wrench cone.
     */
    Eigen::MatrixXd getCWCSpan();
    /** Retrieve the H-representation (Half-space of the polyhedron) of the CWC
     * \return A matrix \f$G\f$ such that \f$Gw_g \leq 0\f$ with \f$w_g\f$ the contact wrench cone.
     */
    Eigen::MatrixXd getCWCFace();

private:
    void resizeG();
    void computeG();
    Eigen::Matrix3d skewMatrix(const Eigen::Vector3d v);
    std::vector<Eigen::Vector3d> generateCone(const ContactSurface& cp);

protected: // protected and not private for python bindings
    Eigen::Vector3d com_;
    std::vector<ContactSurface> cp_;
    Eigen::MatrixXd G_;
    Eigen::Polyhedron polyhedron_;
};

} // namespace wcl
