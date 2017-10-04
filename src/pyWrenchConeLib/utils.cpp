/*      File: utils.cpp
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
#include "utils.h"
#include "PyContactSurface.h"
#include <exception>

namespace py = boost::python;

namespace wcl {
Eigen::Vector3d extractV3d(const np::ndarray& v)
{
    if (v.get_nd() != 1)
        throw std::domain_error("Bad dimension! It should be a matrix of dimension 1.");
    if (v.shape(0) != 3)
        throw std::domain_error("Bad vector size! It should be of length 3.");
    if (v.get_dtype() != np::dtype::get_builtin<double>())
        throw std::domain_error("Bad type! It should be a float.");
    Eigen::Vector3d vOut;
    for (Eigen::Index i = 0; i < 3; ++i)
        vOut(i) = py::extract<double>(v[i]);

    return vOut;
}

Eigen::Matrix3d extractM3d(const np::ndarray& m)
{
    if (m.get_nd() != 2)
        throw std::domain_error("Bad dimension! It should be a matrix of dimension 2.");
    if (m.shape(0) != 3 || m.shape(1) != 3)
        throw std::domain_error("Bad matrix size! It should be of shape 3x3.");
    if (m.get_dtype() != np::dtype::get_builtin<double>())
        throw std::domain_error("Bad type! It should be a float.");
    Eigen::Matrix3d mOut;
    for (Eigen::Index i = 0; i < 3; ++i)
        for (Eigen::Index j = 0; j < 3; ++j)
            mOut(i, j) = py::extract<double>(m[i][j]);

    return mOut;
}

std::vector<ContactSurface> extractCSList(const py::list& py_lpCS)
{
    std::vector<ContactSurface> pCSs;
    for (int i = 0; i < py::len(py_lpCS); ++i) {
        py::extract<PyContactSurface> pCS(py_lpCS[i]);
        if (pCS.check())
            pCSs.emplace_back(pCS);
        else
            throw std::domain_error("It should be a list of PyContactSurface");
    }

    return pCSs;
}

std::vector<Eigen::Vector3d> extractPointsList(const py::list& py_points)
{
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < py::len(py_points); ++i) {
        py::extract<np::ndarray> point(py_points[i]);
        if (point.check())
            points.emplace_back(extractV3d(point));
        else
            throw std::domain_error("It should be a list of numpy array of shape 3.");
    }

    return points;
}

np::ndarray buildNumpyArray(const Eigen::MatrixXd& mat)
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

} // namespace wcl