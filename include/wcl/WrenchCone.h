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

#pragma once

#include "wcl/api.h"
#include "wcl/ContactSurface.h"
#include <eigen-cdd/Polyhedron.h>
#include <vector>

namespace wcl {

/** CWC (Contact Wrench Cone) is the main class of this lib.
 * It computes the Contact Wrench Cone using 6D polyhedrons.
 */
class WCL_DLLAPI WrenchCone {
public:
    /** Constructor
     * \param applicationPoint The application point of the wrench
     * \param cp A contact surface
     */
    WrenchCone(const Eigen::Vector3d& applicationPoint, const ContactSurface& cp);
    /** Constructor
     * \param applicationPoint The application point of the wrench
     * \param cps A list of contact surfaces
     */
    WrenchCone(const Eigen::Vector3d& applicationPoint, const std::vector<ContactSurface>& cps);


    /** Retrieve the V-representation (vertexes and rays of the polyhedron) of the CWC
     * \return A matrix \f$G\f$ such that \f$G\lambda = w_g, \lambda \geq 0\f$ with \f$w_g\f$ the contact wrench cone.
     */
    Eigen::MatrixXd getRays();
    /** Retrieve the H-representation (Half-space of the polyhedron) of the CWC
     * \return A matrix \f$G\f$ such that \f$Gw_g \leq 0\f$ with \f$w_g\f$ the contact wrench cone.
     */
    Eigen::MatrixXd getHalfspaces();

private:
    void resizeG();
    void computeG();
    Eigen::Matrix3d skewMatrix(const Eigen::Vector3d v);
    std::vector<Eigen::Vector3d> generateCone(const ContactSurface& cp);

protected: // protected and not private for python bindings
    Eigen::Vector3d ap_;
    std::vector<ContactSurface> cp_;
    Eigen::MatrixXd G_;
    Eigen::Polyhedron polyhedron_;
};

} // namespace wcl
