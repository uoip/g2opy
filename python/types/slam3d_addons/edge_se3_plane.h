#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d_addons/edge_se3_plane.h>


namespace py = pybind11;
using namespace pybind11::literals;


namespace g2o {

void declareEdgeSE3Plane(py::module & m) {

    templatedBaseBinaryEdge<3, Plane3D, VertexSE3, VertexPlane>(m, "_3_Plane3D_VertexSE3_VertexPlane");
        py::class_<EdgeSE3Plane, BaseBinaryEdge<3, Plane3D, VertexSE3, VertexPlane>>(m, "EdgeSE3Plane")
            .def(py::init<>())

            .def("compute_error", &EdgeSE3Plane::computeError)
            .def("set_measurement", &EdgeSE3Plane::setMeasurement)
        ;

}

}  // end namespace g2o
