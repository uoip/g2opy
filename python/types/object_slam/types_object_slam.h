#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/object_slam/types_object_slam.h>

#include "python/core/base_vertex.h"
#include "python/core/base_binary_edge.h"
#include "python/core/base_unary_edge.h"


namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareTypesObjectSLAM(py::module & m) {

    // TODO Use SBA cam.
    // Project known CAD points from Object frame to world frame, then cam frame
    // and compare to detected kp on image plane.
    // Object SE3 pose projects points from object frame to world frame, 
    // camera SE3 pose projects points from world frame to camera frame.
    templatedBaseBinaryEdge<2, Vector2D, VertexSE3Expmap /* Object */, VertexSE3Expmap /* Cam */>(
            m, "_2_Vector2D_VertexSE3Expmap_VertexSE3Expmap");
    py::class_<EdgeSE3ProjectFromObject, BaseBinaryEdge<2, Vector2D, 
            VertexSE3Expmap, VertexSE3Expmap>>(m, "EdgeSE3ProjectFromObject")
        .def(py::init<>())
        // Args: 
        // cam_k (float64 np.array of [fx, fy, cx, cy]) cam intrinsics
        // p_inO (float64 np.array of size 3) for position of 3D keypointain object frame
        .def(py::init<const Eigen::Vector4d&, const Eigen::Vector3d&>())
        .def("compute_error", &EdgeSE3ProjectFromObject::computeError)
        .def("linearize_oplus", &EdgeSE3ProjectFromObject::linearizeOplus)
        .def("set_info", &EdgeSE3ProjectFromObject::set_info)
    ;
    
    templatedBaseUnaryEdge<2, Vector2D, VertexSE3Expmap /* Cam */>(
            m, "_2_Vector2D_VertexSE3Expmap");
    py::class_<EdgeSE3ProjectFromFixedObject, BaseUnaryEdge<2, Vector2D, 
            VertexSE3Expmap>>(m, "EdgeSE3ProjectFromFixedObject")
        .def(py::init<>())
        // Args: 
        // cam_k (float64 np.array of [fx, fy, cx, cy]) cam intrinsics
        // p_inO (float64 np.array of size 3) for position of 3D keypointain object frame
        .def(py::init<const Eigen::Vector4d&, const Eigen::Vector3d&, 
                const Eigen::Matrix<double,3,4>&>())
        .def("compute_error", &EdgeSE3ProjectFromFixedObject::computeError)
        .def("linearize_oplus", &EdgeSE3ProjectFromFixedObject::linearizeOplus)
        .def("set_info", &EdgeSE3ProjectFromFixedObject::set_info)
    ;
}
}  // end namespace g2o
