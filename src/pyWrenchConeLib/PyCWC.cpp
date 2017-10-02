/*      File: PyCWC.cpp
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
#include "PyCWC.h"
#include "utils.h"
#include <Eigen/Core>
#include <WrenchConeLib/ContactSurface.h>
#include <boost/python.hpp>

namespace wcl {

PyCWC::PyCWC(const np::ndarray& py_com, const PyContactSurface& py_pCS)
    : CWC(extractV3d(py_com), { py_pCS })
{
}

PyCWC::PyCWC(const np::ndarray& py_com, const py::list& py_pCSs)
    : CWC(extractV3d(py_com), extractCSList(py_pCSs))
{
}

np::ndarray PyCWC::getPyRays()
{
    Eigen::MatrixXd G = getRays();
    return buildNumpyArray(G);
}

np::ndarray PyCWC::getPyHalfSpaces()
{
    Eigen::MatrixXd G = CWC::getHalfSpaces();
    return buildNumpyArray(G);
}

np::ndarray PyCWC::buildNumpyArray(const Eigen::MatrixXd& mat)
{
    auto shape = py::make_tuple(mat.rows(), mat.cols()); // Maybe need to have row-major stride
    np::dtype dt = np::dtype::get_builtin<double>();
    np::ndarray mOut = np::empty(shape, dt);
    for (Eigen::Index i = 0; i < mat.rows(); ++i)
        for (Eigen::Index j = 0; j < mat.cols(); ++j)
            mOut[i][j] = mat(i, j);

    // return np::from_data(mat.data(), dt, shape, stride, py::object()); // Couldn't make it work :(
    return mOut;
}

void bindCWC()
{
    std::string doc_PyCWC = "CWC (Contact Wrench Cone) is the main class of this lib. "
                            "It computes the Contact Wrench Cone using 6D polyhedrons"
                            "\n\nparam"
                            "\n\tcom: The center of mass of the system"
                            "\n\tcp: A contact surface";
    std::string doc_PyCWC2 = "CWC (Contact Wrench Cone) is the main class of this lib. "
                             "It computes the Contact Wrench Cone using 6D polyhedrons"
                             "\n\nparam"
                             "\n\tcom: The center of mass of the system"
                             "\n\tcp: A list of contact surfaces";
    std::string doc_getCWCSpan = "Retrieve the V-representation (vertexes and rays of the polyhedron) of the CWC."
                                 "\n\n Return:"
                                 "\n\t A matrix G such that G * lambda = w_g, lambda >= 0 with w_g the contact wrench cone.";
    std::string doc_getCWCFace = "Retrieve the H-representation (Half-space of the polyhedron) of the CWC."
                                 "\n\n Return:"
                                 "\n\t A matrix G such that G *w_g <= 0 with w_g the contact wrench cone.";

    py::class_<PyCWC>("CWC", doc_PyCWC.c_str(), py::init<np::ndarray, PyContactSurface>())
        .def(py::init<np::ndarray, py::list>(doc_PyCWC2.c_str()))
        .def("getRays", &PyCWC::getPyRays, doc_getCWCSpan.c_str())
        .def("getHalfSpaces", &PyCWC::getPyHalfSpaces, doc_getCWCFace.c_str());
}

} // namespace wcl