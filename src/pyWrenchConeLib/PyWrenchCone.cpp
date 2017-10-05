/*      File: PyWrenchCone.cpp
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
#include "PyWrenchCone.h"
#include "utils.h"
#include <Eigen/Core>
#include <wcl/ContactSurface.h>
#include <boost/python.hpp>

namespace wcl {

PyWrenchCone::PyWrenchCone(const np::ndarray& py_com, const PyContactSurface& py_pCS)
    : WrenchCone(extractV3d(py_com), { py_pCS })
{
}

PyWrenchCone::PyWrenchCone(const np::ndarray& py_com, const py::list& py_pCSs)
    : WrenchCone(extractV3d(py_com), extractCSList(py_pCSs))
{
}

np::ndarray PyWrenchCone::pyGetRays()
{
    Eigen::MatrixXd G = getRays();
    return buildNumpyArray(G);
}

np::ndarray PyWrenchCone::pyGetHalfspaces()
{
    Eigen::MatrixXd G = WrenchCone::getHalfspaces();
    return buildNumpyArray(G);
}

void bindWrenchCone()
{
    std::string doc_PyWrenchCone = "WrenchCone (Contact Wrench Cone) is the main class of this lib. "
                                   "It computes the Contact Wrench Cone using 6D polyhedrons";
    std::string doc_PyWrenchConeInit = "Constructor"
                                       "\n\nparam"
                                       "\n\tcom: The center of mass of the system"
                                       "\n\tcp: A contact surface";
    std::string doc_PyWrenchConeInit2 = "Constructor"
                                        "\n\nparam"
                                        "\n\tcom: The center of mass of the system"
                                        "\n\tcp: A list of contact surfaces";
    std::string doc_pyGetRays = "Retrieve the V-representation (vertexes and rays of the polyhedron) of the WrenchCone."
                                "\n\n Return:"
                                "\n\t A matrix G such that G * lambda = w_g, lambda >= 0 with w_g the contact wrench cone.";
    std::string doc_pyGetHalfspaces = "Retrieve the H-representation (Half-space of the polyhedron) of the WrenchCone."
                                      "\n\n Return:"
                                      "\n\t A matrix G such that G *w_g <= 0 with w_g the contact wrench cone.";

    py::class_<PyWrenchCone>("WrenchCone", doc_PyWrenchCone.c_str(), py::init<np::ndarray, PyContactSurface>(doc_PyWrenchConeInit.c_str()))
        .def(py::init<np::ndarray, py::list>(doc_PyWrenchConeInit2.c_str()))
        .def("get_rays", &PyWrenchCone::pyGetRays, doc_pyGetRays.c_str())
        .def("get_halfspaces", &PyWrenchCone::pyGetHalfspaces, doc_pyGetHalfspaces.c_str());
}

} // namespace wcl
