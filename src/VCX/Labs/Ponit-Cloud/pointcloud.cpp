#include "pointcloud.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include <cmath>
#include <glm/gtc/matrix_inverse.hpp>
#include <iostream>
#include <list>
#include <map>
#include <Open3D/Open3D.h>
#include <set>
#include <spdlog/spdlog.h>
#include <unordered_set>

void solve(VCX::Engine::SurfaceMesh & mesh, const char * p) {
    open3d::geometry::PointCloud     pc;
    open3d::io::ReadPointCloudOption po;
    open3d::io::ReadPointCloudFromPCD(p, pc, po);
    pc.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(), false);
    if (pc.HasNormals()) printf("Has normal!\n");
    printf("num points: %ld\n", pc.points_.size());
    int num_points = pc.points_.size();
}
