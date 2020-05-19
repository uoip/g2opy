// refer from : https://raw.githubusercontent.com/koide3/hdl_graph_slam/5447b906f1f3d8eef28021ce15d8d5888d223f9e/include/g2o/edge_se3_plane.hpp

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/edge_se3_plane.h>

namespace g2o {

    EdgeSE3Plane::EdgeSE3Plane() : g2o::BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3, g2o::VertexPlane>() {

    }

    bool EdgeSE3Plane::read(std::istream& is) {
        Eigen::Vector4d v;
        is >> v(0) >> v(1) >> v(2) >> v(3);
        setMeasurement(Plane3D(v));
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;    
    }

    bool EdgeSE3Plane::write(std::ostream& os) const{
        Eigen::Vector4d v = _measurement.toVector();
        os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }

    void EdgeSE3Plane::computeError() {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);

        Eigen::Isometry3d w2n = v1->estimate().inverse();
        Plane3D local_plane = w2n * v2->estimate();
        _error = local_plane.ominus(_measurement);
    }
}

