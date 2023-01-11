#include "pointcloud.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "utils.h"
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

struct Grid {
    double                             r;
    const PCD_t *                      pc;
    std::vector<std::vector<uint32_t>> lst;
    glm::vec3                          PMIN, PMAX;
    uint32_t                           n[3];
    Grid(double r, PCD_t & pcd) {
        this->pc  = &pcd;
        this->r   = r;
        this->lst = std::vector<std::vector<uint32_t>>();
        PMIN      = glm::vec3(1e9);
        PMAX      = glm::vec3(-1e9);
        for (auto p : pcd.points_) {
            auto v = eigen2glm(p);
            for (int i = 0; i < 3; i++) {
                PMIN[i] = min(PMIN[i], v[i]);
                PMAX[i] = max(PMAX[i], v[i]);
            }
        }
        for (int i = 0; i < 3; i++) this->n[i] = uint32_t((PMAX - PMIN)[i] / r);
        for (int i = 0; i < this->n[0]; i++) {
            for (int j = 0; j < this->n[1]; j++) {
                for (int k = 0; k < this->n[2]; k++) {
                    this->lst.push_back(std::vector<uint32_t>());
                }
            }
        }
        for (int j = 0; j < pcd.points_.size(); j++) {
            auto     v = eigen2glm(pcd.points_[j]);
            uint32_t m[3];
            for (int i = 0; i < 3; i++) m[i] = uint32_t((v - PMIN)[i] / r);
            this->lst[GET_INDX(m[0], m[1], m[2], n)].push_back(j);
        }
    }
};

void BPA(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh) {
    auto   distances = pc.ComputeNearestNeighborDistance();
    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double              rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { 0.05 * rho, 0.1 * rho, 0.2 * rho, 0.5 * rho,
                                rho,        2.0 * rho, 4.0 * rho, 8 * rho };
}

void solve(
    const VCX::Engine::SurfaceMesh & old, VCX::Engine::SurfaceMesh & mesh, const char * path) {
    open3d::geometry::PointCloud pc;
    Grid(0.001, pc);
    Mesh2PC(old, pc);
    LibAlg(pc, mesh, 1);
}