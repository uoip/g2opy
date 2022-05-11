#ifndef G2O_EDGE_SE3_PLANE_H_
#define G2O_EDGE_SE3_PLANE_H_
// refer from : https://raw.githubusercontent.com/koide3/hdl_graph_slam/5447b906f1f3d8eef28021ce15d8d5888d223f9e/include/g2o/edge_se3_plane.hpp

#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam3d_addons_api.h"

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {

	class EdgeSE3Plane : public g2o::BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3, g2o::VertexPlane> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		EdgeSE3Plane();
		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		void computeError();

		virtual void setMeasurement(const g2o::Plane3D& m){
			_measurement = m;
		}


	};
}

#endif
