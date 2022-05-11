#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <g2o/types/slam3d_addons/plane3d.h>
#include "python/core/base_vertex.h"
#include "python/core/base_edge.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declarePlane3D(py::module & m){

    py::class_<Plane3D>(m, "Plane3D")
        .def(py::init<>())
        .def(py::init<const Eigen::Vector4d&>(),
            "v"_a)
        
        .def("to_vector", &Plane3D::toVector)
        .def("coeffs", &Plane3D::coeffs)
        .def("from_vector", &Plane3D::fromVector,
            "coeffs"_a)
        .def_static("azimuth", &Plane3D::azimuth,
            "v"_a)
        .def_static("elevation", &Plane3D::elevation,
            "v"_a)
        .def("distance", &Plane3D::distance)
        .def("normal", &Plane3D::normal)

        .def_static("rotation", &Plane3D::rotation,
            "v"_a)
        .def("oplus", &Plane3D::oplus,
            "v"_a)
        .def("ominus", &Plane3D::ominus,
            "plane"_a)

        .def_static("normalize", &Plane3D::normalize,
            "coeffs"_a)

        // operator
        .def(Eigen::Isometry3d() * py::self)
    ;
    templatedBaseVertex<3, Plane3D>(m, "_3_Plane3D");
    templatedBaseEdge<3, Plane3D>(m, "_3_Plane3D");
    templatedBaseMultiEdge<3, Plane3D>(m, "_3_Plane3D");
        
}

}  // end namespace g2o