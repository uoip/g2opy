# https://github.com/gaoxiang12/slambook2/blob/master/ch10/pose_graph_g2o_SE3.cpp
# Original From gaoxiang12
# Modified By Yannnnnnnnnnnn

import numpy as np
import g2o
import os

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement, 
            information=np.identity(6),
            robust_kernel=None):

        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()


pose_graph = PoseGraphOptimization()
pose_file = open('sphere.g2o')
line = pose_file.readline()

while line:

    data = line.split(' ')
    
    if( data[0]=='VERTEX_SE3:QUAT' ):

        pose_id   = int(data[1])
        pose_info = np.array( [float(i) for i in data[2:9]] )

        # print(pose_id,pose_info)
        
        q = g2o.Quaternion(pose_info[6],pose_info[3],pose_info[4],pose_info[5])
        t = g2o.Isometry3d(q,pose_info[0:3])

        pose_graph.add_vertex(pose_id,t)

    if( data[0]=='EDGE_SE3:QUAT' ):

        pose_id_left = int(data[1])
        pose_id_righ = int(data[2])
        pose_info = np.array( [float(i) for i in data[3:10]] )


        q = g2o.Quaternion(pose_info[6],pose_info[3],pose_info[4],pose_info[5])
        t = g2o.Isometry3d(q,pose_info[0:3])

        pose_graph.add_edge([pose_id_left,pose_id_righ],t)

    line = pose_file.readline()

pose_file.close()

pose_graph.optimize(20)
pose_graph.save('result.g2o')
