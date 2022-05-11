#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <python/core/base_edge.h>
#include <g2o/types/slam3d_addons/edge_plane.h>
#include <g2o/types/slam3d_addons/edge_se3_plane_calib.h>


namespace py = pybind11;
using namespace pybind11::literals;


namespace g2o {

void declareEdgePlane(py::module & m) {

    templatedBaseBinaryEdge<4, Vector4D, VertexPlane, VertexPlane>(m, "_4_Vector4D_VertexPlane_VertexPlane");
        py::class_<EdgePlane, BaseBinaryEdge<4, Vector4D, VertexPlane, VertexPlane>>(m, "EdgePlane")
            .def(py::init<>())

            .def("compute_error", &EdgePlane::computeError)
            .def("set_measurement", &EdgePlane::setMeasurement)
            .def("set_measurement_data", &EdgePlane::setMeasurementData)
            .def("get_measurement_data", &EdgePlane::getMeasurementData)
            .def("measurement_dimension", &EdgePlane::measurementDimension)
            .def("set_measurement_from_state", &EdgePlane::setMeasurementFromState)
            .def("initial_estimate_possible", &EdgePlane::initialEstimatePossible)

        ;

}

}  // end namespace g2o
