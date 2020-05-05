#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d_addons/plane3d.h>


namespace py = pybind11;
using namespace pybind11::literals;


namespace g2o {

void declareVertexPlane(py::module & m) {

    py::class_<VertexPlane, BaseVertex<3, Plane3D>>(m, "VertexPlane")
        .def(py::init<>())

        .def("set_to_origin_impl", &VertexPlane::setToOriginImpl)
        .def("oplus_impl", &VertexPlane::oplusImpl)
        .def("set_estimate_data_impl", &VertexPlane::setEstimateDataImpl)
        .def("get_estimate_data", &VertexPlane::getEstimateData)
        .def("estimate_dimension", &VertexPlane::estimateDimension)
    ;


    /*
    py::class_<VertexSE3WriteGnuplotAction, WriteGnuplotAction>(m, "VertexSE3WriteGnuplotAction")
        .def(py::init<>())
        .def("__call__", &VertexSE3WriteGnuplotAction::operator())
    ;

    // #ifdef G2O_HAVE_OPENGL
    py::class_<VertexSE3DrawAction, DrawAction>(m, "VertexSE3DrawAction")
        .def(py::init<>())
        .def("__call__", &VertexSE3DrawAction::operator())
    ;
    // #endif
    */

}

}  // end namespace g2o