#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "pointcloud.h"
#include <cmath>
#include <glm/gtc/matrix_inverse.hpp>
#include <iostream>
#include <list>
#include <map>
#include <open3d/Open3D.h>
#include <set>
#include <spdlog/spdlog.h>
#include <tuple>
#include <unordered_set>
typedef open3d::geometry::PointCloud PCD_t;
#define GET_INDX(i, j, k, n) (i * n[1] * n[2] + j * n[2] + k)
void Mesh2PC(const VCX::Engine::SurfaceMesh & mesh, open3d::geometry::PointCloud & pc);

void LoadData(open3d::geometry::PointCloud & pc, const std::string path);

void LibAlg(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh, int type = 0);

glm::vec3       eigen2glm(Eigen::Vector3d a);
Eigen::Vector3d glm2eigen(glm::vec3 a);
void            BPA_run(
               const VCX::Engine::SurfaceMesh & old,
               open3d::geometry::PointCloud &   pc,
               VCX::Engine::SurfaceMesh &       mesh);