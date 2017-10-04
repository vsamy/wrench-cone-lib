/*      File: PyContactSurface.cpp
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
#include "PyContactSurface.h"
#include "utils.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/python.hpp>
#include <iostream>
#include <string>

namespace py = boost::python;

namespace wcl {

PyContactSurface::PyContactSurface(const np::ndarray& py_r_0_s, const np::ndarray& py_E_0_s,
    const py::list& py_points, double py_mu, unsigned int py_nrGenerators)
{
    r_0_s = extractV3d(py_r_0_s);
    E_0_s = extractM3d(py_E_0_s);
    points = extractPointsList(py_points);
    mu = py_mu;
    nrGenerators = py_nrGenerators;
}

PyContactSurface pyRectangularSurface(double py_xLength, double py_yLength, const np::ndarray& py_r_0_s,
    const np::ndarray& py_E_0_s, double py_mu, unsigned int py_nrGenerators)
{
    std::vector<Eigen::Vector3d> p;
    p.emplace_back(py_xLength, py_yLength, 0);
    p.emplace_back(py_xLength, -py_yLength, 0);
    p.emplace_back(-py_xLength, -py_yLength, 0);
    p.emplace_back(-py_xLength, py_yLength, 0);

    PyContactSurface pCS(py_r_0_s, py_E_0_s, py::list{}, py_mu, py_nrGenerators);
    pCS.points = p;
    return pCS;
}

// Create wrappers
PyContactSurface pyRectangularSurface0(double py_xHalfLength, double py_yHalfLength, const np::ndarray& py_r_0_s, const np::ndarray& py_E_0_s)
{
    return pyRectangularSurface(py_xHalfLength, py_yHalfLength, py_r_0_s, py_E_0_s);
}
PyContactSurface pyRectangularSurface1(double py_xHalfLength, double py_yHalfLength, const np::ndarray& py_r_0_s, const np::ndarray& py_E_0_s, double py_mu)
{
    return pyRectangularSurface(py_xHalfLength, py_yHalfLength, py_r_0_s, py_E_0_s, py_mu);
}

void bindContactSurface()
{
    std::string doc_PyContactSurface = "Representation of a contact surface."
                                       "\n\nparam:"
                                       "\n\tr_0_s: Position vector of the surface in the world coordinates"
                                       "\n\tE_0_s: Rotation matrix of the surface from the world frame to the surface frame"
                                       "\n\tpoints: List of points belonging to the surface in the surface coordinates"
                                       "\n\tmu: Static friction coefficient of the surface"
                                       "\n\tnrGenerators: Number of generators to approximate the friction cone";
    std::string doc_mu = "Static friction coefficient of the surface";
    std::string doc_nrGenerators = "Number of generators to approximate the friction cone";
    std::string doc_pyRectangularSurface0 = "Function that build a rectangular surface. "
                                            "The z-axis of the surface is its normal. "
                                            "The x-axis, y-axis and z-axis are defined with regard to the given rotation matrix E_0_s."
                                            "\n\nparam:"
                                            "\n\txHalfLength: The half-length of the surface on the x-axis"
                                            "\n\tyHalfLength: The half-length of the surface on the y-axis"
                                            "\n\tr_0_s: Position vector of the surface in the world coordinates"
                                            "\n\tE_0_s: Rotation matrix of the surface from the world frame to the surface frame";
    std::string doc_pyRectangularSurface1 = doc_pyRectangularSurface0 + "\n\tmu: Static friction coefficient of the surface";
    std::string doc_pyRectangularSurface = doc_pyRectangularSurface1 + "\n\tnrGenerators: Number of generators to approximate the friction cone";

    py::class_<PyContactSurface>("ContactSurface", doc_PyContactSurface.c_str(),
      py::init<np::ndarray, np::ndarray, py::list, py::optional<double, unsigned int> >())
        .def_readwrite("mu", &PyContactSurface::mu, doc_mu.c_str())
        .def_readwrite("nrGenerators", &PyContactSurface::nrGenerators, doc_nrGenerators.c_str());

    // BOOST_PYTHON_FUNCTION_OVERLOADS(pySquareSurfacePoints_overloads, pySquareSurfacePoints, 4, 6) // Do not work and i don't know why
    py::def("rectangularSurface", pyRectangularSurface0, doc_pyRectangularSurface0.c_str());
    py::def("rectangularSurface", pyRectangularSurface1, doc_pyRectangularSurface1.c_str());
    py::def("rectangularSurface", pyRectangularSurface, doc_pyRectangularSurface.c_str());
}

} // namespace wcl
