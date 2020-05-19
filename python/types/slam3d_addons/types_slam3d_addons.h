#include <pybind11/pybind11.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>
#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

// #include "se3quat.h"
// #include "vertex_se3.h"
// #include "vertex_pointxyz.h"

// #include "edge_pointxyz.h"
// #include "edge_se3.h"
// #include "edge_se3_pointxyz.h"
#include "plane3d.h"
#include "vertex_plane.h"
#include "edge_plane.h"
#include "edge_se3_plane.h"


namespace g2o {


// register types
// slam3d_addons
G2O_REGISTER_TYPE_GROUP(slam3d_addons);
G2O_REGISTER_TYPE(VERTEX3, VertexSE3Euler);
G2O_REGISTER_TYPE(EDGE3, EdgeSE3Euler);
G2O_REGISTER_TYPE(VERTEX_PLANE, VertexPlane);
G2O_REGISTER_TYPE(EDGE_SE3_PLANE_CALIB, EdgeSE3PlaneSensorCalib);

G2O_REGISTER_TYPE(VERTEX_LINE3D, VertexLine3D);
G2O_REGISTER_TYPE(EDGE_SE3_LINE3D, EdgeSE3Line3D);
G2O_REGISTER_TYPE(EDGE_PLANE, EdgePlane);
G2O_REGISTER_TYPE(EDGE_SE3_CALIB, EdgeSE3Calib);
    

void declareTypesSlam3dAddons(py::module & m) {

    declarePlane3D(m);

    declareVertexPlane(m);
    declareEdgePlane(m);
    declareEdgeSE3Plane(m);


}

}