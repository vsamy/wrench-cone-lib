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

#include "wcl/ContactSurface.h"
#include <vector>

namespace wcl {

wcl::ContactSurface rectangularSurface(double xHalfLength, double yHalfLength, const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotation, double mu, unsigned int nrGenerators)
{
    wcl::ContactSurface cs;
    cs.points.emplace_back(xHalfLength, yHalfLength, 0);
    cs.points.emplace_back(xHalfLength, -yHalfLength, 0);
    cs.points.emplace_back(-xHalfLength, -yHalfLength, 0);
    cs.points.emplace_back(-xHalfLength, yHalfLength, 0);
    cs.position = position;
    cs.rotation = rotation;
    cs.mu = mu;
    cs.nrGenerators = nrGenerators;
    return cs;
}

} // namespace wcl
