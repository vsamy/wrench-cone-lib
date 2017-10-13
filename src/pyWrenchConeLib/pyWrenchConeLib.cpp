/*      File: pyWrenchConeLib.cpp
*       This file is part of the program wrench-cone-lib
*       Program description : This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. thon bindings are also available.
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
// #include "WrenchCone.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <memory>
#include <pygen/converters.h>
#include <wcl/ContactSurface.h>
#include <wcl/WrenchCone.h>

#include <iostream>

namespace py = boost::python;
namespace np = boost::python::numpy;

struct python_list_of_list_to_std_vector_of_eigen_Vector3d {
    python_list_of_list_to_std_vector_of_eigen_Vector3d()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<std::vector<Eigen::Vector3d> >());
    }

    static void* convertible(PyObject* obj_ptr)
    {

        if (!PySequence_Check(obj_ptr))
            return 0;

        py::list arr = py::extract<py::list>(obj_ptr);
        for (long i = 0; i < py::len(arr); ++i) {
            py::extract<py::list> extract(arr[i]);
            if (!extract.check() || py::len(extract()) != 3)
                return 0;
            else
                for (int j = 0; j < 3; ++j)
                    if (!py::extract<double>(extract()[j]).check())
                        return 0;
        }

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list list = py::extract<py::list>(obj_ptr);
        long len = py::len(list);

        using storage_type = py::converter::rvalue_from_python_storage<std::vector<Eigen::Vector3d> >;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) std::vector<Eigen::Vector3d>;
        std::vector<Eigen::Vector3d>& vec = *static_cast<std::vector<Eigen::Vector3d>*>(storage);

        for (long i = 0; i < len; ++i) {
            py::list l = py::extract<py::list>(list[i]);
            vec.emplace_back(py::extract<double>(l[0]), py::extract<double>(l[1]), py::extract<double>(l[2]));
        }

        data->convertible = storage;
    }
};

// List of numpy array to std::vector<Eigen::MatrixXd>
struct python_list_to_std_vector_of_eigen_Vector3d {
    python_list_to_std_vector_of_eigen_Vector3d()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<std::vector<Eigen::Vector3d> >());
    }

    static void* convertible(PyObject* obj_ptr)
    {

        if (!PySequence_Check(obj_ptr))
            return 0;

        py::list arr = py::extract<py::list>(obj_ptr);
        for (long i = 0; i < py::len(arr); ++i) {
            py::extract<np::ndarray> extract(arr[i]);
            if (!extract.check() || extract().get_nd() != 1 || extract().shape(0) != 3)
                return 0;
            else
                for (int j = 0; j < 3; ++j)
                    if (!py::extract<double>(extract()[j]).check())
                        return 0;
        }

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list list = py::extract<py::list>(obj_ptr);
        long len = py::len(list);

        using storage_type = py::converter::rvalue_from_python_storage<std::vector<Eigen::Vector3d> >;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) std::vector<Eigen::Vector3d>;
        std::vector<Eigen::Vector3d>& vec = *static_cast<std::vector<Eigen::Vector3d>*>(storage);

        for (long i = 0; i < len; ++i) {
            np::ndarray v = py::extract<np::ndarray>(list[i]);
            vec.emplace_back(py::extract<double>(v[0]), py::extract<double>(v[1]), py::extract<double>(v[2]));
        }

        data->convertible = storage;
    }
};

struct std_vector_of_eigen_Vector3d_to_python_list {
    static PyObject* convert(const std::vector<Eigen::Vector3d>& vec)
    {
        py::list list;

        auto buildArray = [](const Eigen::Vector3d& vec) {
            auto shape = py::make_tuple(3);
            np::dtype dt = np::dtype::get_builtin<double>();
            np::ndarray arr = np::empty(shape, dt);
            for (Eigen::Index i = 0; i < vec.rows(); ++i)
                arr[i] = vec(i);

            return arr;

        };

        for (const Eigen::Vector3d& v : vec) {
            list.append(buildArray(v));
        }

        return py::incref(list.ptr());
    }
};

namespace wcl {
// ContactSurface struct with getter and setter
struct PyContactSurface : public ContactSurface {
    PyContactSurface(const Eigen::Vector3d& Py_position, const Eigen::Matrix3d& Py_rotation, std::vector<Eigen::Vector3d> Py_points,
        double Py_mu = 0.7, unsigned int Py_nrGenerators = 4)
    {
        position = Py_position;
        rotation = Py_rotation;
        points = Py_points;
        mu = Py_mu;
        nrGenerators = Py_nrGenerators;
    }

    Eigen::Vector3d get_position() { return position; }
    void set_position(const Eigen::Vector3d& pos) { position = pos; }
    Eigen::Matrix3d get_rotation() { return rotation; }
    void set_rotation(const Eigen::Matrix3d& rot) { rotation = rot; }
    std::vector<Eigen::Vector3d> get_points() { return points; }
    void set_points(const std::vector<Eigen::Vector3d>& ps) { points = ps; }
};

// Wrench that can get PyContactSurface
class PyWrenchCone : public WrenchCone {
public:
    using WrenchCone::WrenchCone;
    PyWrenchCone(const Eigen::Vector3d& applicationPoint, const std::vector<PyContactSurface>& cps)
        : WrenchCone(applicationPoint, std::vector<ContactSurface>(cps.cbegin(), cps.cend()))
    {
    }
};

// Create wrappers
PyContactSurface PyRectangularSurface(double xHalfLength, double yHalfLength, const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotation, double mu = 0.7, unsigned int nrGenerators = 4)
{
    std::vector<Eigen::Vector3d> p;
    p.emplace_back(xHalfLength, yHalfLength, 0);
    p.emplace_back(xHalfLength, -yHalfLength, 0);
    p.emplace_back(-xHalfLength, -yHalfLength, 0);
    p.emplace_back(-xHalfLength, yHalfLength, 0);
    return PyContactSurface(position, rotation, p, mu, nrGenerators);
}
PyContactSurface PyRectangularSurface0(double xHalfLength, double yHalfLength, const Eigen::Vector3d& r_0_s, const Eigen::Matrix3d& E_0_s)
{
    return PyRectangularSurface(xHalfLength, yHalfLength, r_0_s, E_0_s);
}
PyContactSurface PyRectangularSurface1(double xHalfLength, double yHalfLength, const Eigen::Vector3d& r_0_s, const Eigen::Matrix3d& E_0_s, double mu)
{
    return PyRectangularSurface(xHalfLength, yHalfLength, r_0_s, E_0_s, mu);
}

struct python_list_to_std_vector_of_ContactSurface {
    python_list_to_std_vector_of_ContactSurface()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<std::vector<PyContactSurface> >());
    }

    static void* convertible(PyObject* obj_ptr)
    {

        if (!PySequence_Check(obj_ptr))
            return 0;

        py::list arr = py::extract<py::list>(obj_ptr);
        for (long i = 0; i < py::len(arr); i++)
            if (!py::extract<PyContactSurface>(arr[i]).check())
                return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list list = py::extract<py::list>(obj_ptr);
        long len = py::len(list);

        using storage_type = py::converter::rvalue_from_python_storage<std::vector<PyContactSurface> >;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) std::vector<PyContactSurface>;
        std::vector<PyContactSurface>& vec = *static_cast<std::vector<PyContactSurface>*>(storage);

        for (long i = 0; i < len; ++i)
            vec.push_back(py::extract<PyContactSurface>(list[i]));

        data->convertible = storage;
    }
};

struct std_vector_of_ContactSurface_to_python_list {
    static PyObject* convert(const std::vector<PyContactSurface>& vec)
    {
        py::list list;

        for (const PyContactSurface& cs : vec)
            list.append(cs);

        return py::incref(list.ptr());
    }
};

} // namespace wcl

BOOST_PYTHON_MODULE(WrenchConeLib)
{
    Py_Initialize();
    np::initialize();

    using namespace wcl;
    using namespace pygen;
    convertMatrix<Eigen::Matrix3d>();
    convertMatrix<Eigen::MatrixXd>();
    convertVector<Eigen::Vector3d>();

    std::string doc_WrenchCone = "WrenchCone (Contact Wrench Cone) is the main class of this lib. "
                                 "It computes the Contact Wrench Cone using 6D polyhedrons";
    std::string doc_getRays = "Retrieve the V-representation (vertexes and rays of the polyhedron) of the WrenchCone.";
    std::string doc_getHalfspaces = "Retrieve the H-representation (Half-space of the polyhedron) of the WrenchCone.";
    std::string doc_ContactSurface = "Representation of a contact surface.";
    std::string doc_mu = "Static friction coefficient of the surface";
    std::string doc_nrGenerators = "Number of generators to approximate the friction cone";
    std::string doc_rectangularSurface = "Function that build a rectangular surface. "
                                         "The z-axis of the surface is its normal. "
                                         "The x-axis, y-axis and z-axis are defined with regard to the given rotation matrix E_0_s.";
    std::string doc_position = "Current position of the surface in the world coordinates";
    std::string doc_rotation = "Current rotation of the surface in the world frame";
    std::string doc_points = "Current points of the surface in the surface frame";

    // Conversion of std::vector<Eigen::Vector3d>
    python_list_of_list_to_std_vector_of_eigen_Vector3d();
    python_list_to_std_vector_of_eigen_Vector3d();
    py::to_python_converter<std::vector<Eigen::Vector3d>, std_vector_of_eigen_Vector3d_to_python_list>();

    // ContactSurface
    py::class_<PyContactSurface>("ContactSurface", doc_ContactSurface.c_str(),
        py::init<Eigen::Vector3d, Eigen::Matrix3d, std::vector<Eigen::Vector3d>, py::optional<double, unsigned int> >(py::args("position", "rotation", "points", "mu", "nr_generators")))
        .def_readwrite("mu", &PyContactSurface::mu, doc_mu.c_str())
        .def_readwrite("nr_generators", &PyContactSurface::nrGenerators, doc_nrGenerators.c_str())
        .add_property("position", &PyContactSurface::get_position, &PyContactSurface::set_position, doc_position.c_str())
        .add_property("rotation", &PyContactSurface::get_rotation, &PyContactSurface::set_rotation, doc_rotation.c_str())
        .add_property("points", &PyContactSurface::get_points, &PyContactSurface::set_points, doc_points.c_str());

    // BOOST_PYTHON_FUNCTION_OVERLOADS(pySquareSurfacePoints_overloads, pySquareSurfacePoints, 4, 6) // Do not work and i don't know why
    py::def("rectangular_surface", PyRectangularSurface0, py::args("x_half_length", "y_half_length", "position", "rotation"), doc_rectangularSurface.c_str());
    py::def("rectangular_surface", PyRectangularSurface1, py::args("x_half_length", "y_half_length", "position", "rotation", "mu"), doc_rectangularSurface.c_str());
    py::def("rectangular_surface", PyRectangularSurface, py::args("x_half_length", "y_half_length", "position", "rotation", "mu", "nr_generators"), doc_rectangularSurface.c_str());

    // Conversion of std::vector<ContactSurface>
    python_list_to_std_vector_of_ContactSurface();
    py::to_python_converter<std::vector<PyContactSurface>, std_vector_of_ContactSurface_to_python_list>();

    // WrenchCone
    py::class_<PyWrenchCone>("WrenchCone", doc_WrenchCone.c_str(), py::init<Eigen::Vector3d, ContactSurface>(py::args("application_point", "contact_surface")))
        .def(py::init<Eigen::Vector3d, std::vector<PyContactSurface> >(py::args("application_point", "list_of_contact_surfaces")))
        .def("get_rays", &WrenchCone::getRays, doc_getRays.c_str())
        .def("get_halfspaces", &WrenchCone::getHalfspaces, doc_getHalfspaces.c_str());

    py::implicitly_convertible<PyContactSurface, ContactSurface>();
}
