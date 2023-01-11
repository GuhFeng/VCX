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
    void                               load(double r, PCD_t & pcd) {
        if (! lst.empty()) lst.clear();
        this->pc   = &pcd;
        this->r    = r;
        this->PMIN = glm::vec3(1e9);
        this->PMAX = glm::vec3(-1e9);
        for (auto p : pcd.points_) {
            auto v = eigen2glm(p);
            for (int i = 0; i < 3; i++) {
                PMIN[i] = min(PMIN[i], v[i]);
                PMAX[i] = max(PMAX[i], v[i]);
            }
        }
        for (int i = 0; i < 3; i++) this->n[i] = uint32_t((PMAX - PMIN)[i] / r);
        for (int i = 0; i < this->n[0] + 1; i++) {
            for (int j = 0; j < this->n[1] + 1; j++) {
                for (int k = 0; k < this->n[2] + 1; k++) {
                    this->lst.emplace_back(std::vector<uint32_t>());
                }
            }
        }
        for (int j = 0; j < pcd.points_.size(); j++) {
            auto     v = eigen2glm(pcd.points_[j]);
            uint32_t m[3];
            for (int i = 0; i < 3; i++) m[i] = uint32_t((v - PMIN)[i] / r);
            this->lst[GET_INDX(m[0], m[1], m[2], this->n)].push_back(j);
        }
    }
    std::vector<uint32_t> Get_Neighbor(glm::vec3 v) {
        uint32_t m[3];
        for (int i = 0; i < 3; i++) m[i] = uint32_t((v - PMIN)[i] / r);
        std::vector<uint32_t> lst_n;
        for (int i = max(0, m[0] - 1); i < min(m[0] + 1, n[0]); i++) {
            for (int j = max(0, m[1] - 1); j < min(m[1] + 1, n[1]); j++) {
                for (int k = max(0, m[2] - 1); k < min(m[2] + 1, n[2]); k++) {
                    if (this->lst[GET_INDX(i, j, k, this->n)].empty()) continue;
                    for (int cnt = 0; cnt < this->lst[GET_INDX(i, j, k, this->n)].size(); cnt++) {
                        lst_n.push_back(this->lst[GET_INDX(i, j, k, this->n)][cnt]);
                    }
                }
            }
        }
        return lst_n;
    }
};

struct Edges {
    uint32_t v1, v2, v_op;
    uint32_t tri, onfront;
    Edges() {
        tri     = 1;
        onfront = 1;
    }
};

struct Front {
    std::vector<Edges> edges;
    std::set<uint32_t> act_edges;
    std::set<uint32_t> act_vtx;
};

bool get_center(
    glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, double r, std::vector<glm::vec3> & center) {
    glm::vec3 n    = glm::cross(v2 - v1, v3 - v1);
    n              = glm::normalize(n);
    glm::vec3 left = glm::cross(
        glm::vec3(glm::length(v3 - v1) * glm::length(v3 - v1)) * (glm::cross(v2 - v1, v3 - v1)),
        (v2 - v1));
    glm::vec3 right = glm::cross(
        glm::vec3(glm::length(v2 - v1) * glm::length(v2 - v1)) * (glm::cross(v3 - v1, v2 - v1)),
        (v3 - v1));
    double denominator =
        (2.0 * glm::length(glm::cross(v2 - v1, v3 - v1))
         * glm::length(glm::cross(v2 - v1, v3 - v1)));
    glm::vec3 v0         = v1 + (left + right) * glm::vec3(1 / denominator);
    double    min_radius = glm::length(v1 - v0);
    center.clear();
    if (r >= min_radius) {
        double theta = glm::acos(min_radius / r);
        double t     = glm::sin(theta) * r;
        center.push_back(v0 + n * glm::vec3(t));
        center.push_back(v0 - n * glm::vec3(t));
        return 1;
    }
    return 0;
}

struct BPA {
    std::set<uint32_t>    used_vtx;
    Front                 front;
    std::vector<uint32_t> triangles;
    std::vector<double>   radii;
    Grid                  grid;
    PCD_t &               pc;
    BPA(PCD_t & pcd, std::vector<double> & r): radii(r), pc(pcd) {}
};

void BPA_run(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh) {
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
    Mesh2PC(old, pc);
    LibAlg(pc, mesh, 1);
}