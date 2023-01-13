#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "pointcloud.h"
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
        this->PMIN = glm::vec3(1e5);
        this->PMAX = glm::vec3(-1e5);
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
    std::vector<uint32_t> Get_Neighbor(glm::vec3 v, double rad) {
        uint32_t m[3];
        if (rad > r) printf("warning: the rad is too large!\n");
        for (int i = 0; i < 3; i++) m[i] = uint32_t((v - PMIN)[i] / r);
        std::vector<uint32_t> lst_n, lst_n_tmp;
        for (int i = m[0] - 1; i < m[0] + 2; i++) {
            for (int j = m[1] - 1; j < m[1] + 2; j++) {
                for (int k = m[2] - 1; k < m[2] + 2; k++) {
                    if (this->lst[GET_INDX(i, j, k, this->n)].empty()) continue;
                    for (int cnt = 0; cnt < this->lst[GET_INDX(i, j, k, this->n)].size(); cnt++) {
                        lst_n_tmp.push_back(this->lst[GET_INDX(i, j, k, this->n)][cnt]);
                        if (glm::length(
                                v
                                - eigen2glm(
                                    pc->points_[this->lst[GET_INDX(i, j, k, this->n)][cnt]]))
                            <= rad + 1e-6)
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
    Edges(uint32_t p1, uint32_t p2, uint32_t po) {
        v1      = p1;
        v2      = p2;
        v_op    = po;
        tri     = 1;
        onfront = 1;
    }
    bool operator<(const Edges & e) const {
        if ((v1 + v2) != (e.v1 + e.v2)) return (v1 + v2) < (e.v1 + e.v2);
        if (abs(int(v1 - v2)) != abs(int(e.v1 - e.v2)))
            return abs(int(v1 - v2)) < abs(int(e.v1 - e.v2));
        return v_op < e.v_op;
    }
};

struct Front {
    std::set<Edges> act_edges;
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
        center.push_back(n);
        return 1;
    }
    return 0;
}

struct BPA {
    std::set<uint32_t>    used_vtx;
    Front                 front;
    std::vector<uint32_t> triangles;
    std::vector<double>   radii;
    double                r_now;
    uint32_t              indx;
    Grid                  grid;
    PCD_t &               pc;
    BPA(PCD_t & pcd, std::vector<double> & r): radii(r), pc(pcd) {
        r_now = r[0];
        indx  = 0;
        grid.load(2.0 * r_now, pcd);
    }
    bool valid_ball(glm::vec3 c, glm::vec3 dir, glm::vec3 norm) {
        auto neibors = grid.Get_Neighbor(c, r_now);
        if (neibors.size() == 3 && dot(dir, norm) <= 0) return 1;
        return 0;
    }
    bool find_seed() {
        for (int p = 0; p < pc.points_.size(); p++) {
            glm::vec3 v = eigen2glm(pc.points_[p]), nor = eigen2glm(pc.normals_[p]);
            if (used_vtx.count(p)) continue;
            auto neibors = grid.Get_Neighbor(v, 2.0 * r_now);
            for (int i = 0; i < neibors.size(); i++) {
                for (int j = i + 1; j < neibors.size(); j++) {
                    glm::vec3 v1 = eigen2glm(pc.points_[neibors[i]]),
                              v2 = eigen2glm(pc.points_[neibors[j]]);
                    uint32_t p1 = neibors[i], p2 = neibors[j];
                    if (p1 == p || p2 == p) continue;
                    std::vector<glm::vec3> center;
                    if (! get_center(v1, v2, v, r_now, center)) continue;
                    auto center1 = center[0], center2 = center[1], unit_dir = center[2];
                    if ((! valid_ball(center1, unit_dir, nor))
                        && (! valid_ball(center2, -unit_dir, nor)))
                        continue;
                    Edges e1(p, p1, p2), e2(p1, p, p2), e3(p1, p2, p);
                    triangles.push_back(p);
                    triangles.push_back(p1);
                    triangles.push_back(p2);
                    front.act_edges.insert(e1);
                    front.act_edges.insert(e2);
                    front.act_edges.insert(e3);
                    used_vtx.insert(p);
                    used_vtx.insert(p1);
                    used_vtx.insert(p2);
                    return 1;
                }
            }
            used_vtx.insert(p);
        }
        return 0;
    }
    void pivot_ball(Edges & e) {
        glm::vec3 mid       = eigen2glm((pc.points_[e.v1] + pc.points_[e.v2]) / 2.0);
        auto      neighbors = grid.Get_Neighbor(mid, 2.0 * r_now);
        auto      v1 = eigen2glm(pc.points_[e.v1]), v2 = eigen2glm(pc.points_[e.v2]);
        for (uint32_t num : neighbors) {
            if (num == e.v_op || num == e.v1 || num == e.v2) continue;
            auto                   vtx = eigen2glm(pc.points_[num]);
            auto                   nor = eigen2glm(pc.normals_[num]);
            std::vector<glm::vec3> center;
            if (! get_center(v1, v2, vtx, r_now, center)) continue;
            auto center1 = center[0], center2 = center[1], unit_dir = center[2];
            if ((! valid_ball(center1, unit_dir, nor)) && (! valid_ball(center2, -unit_dir, nor)))
                continue;
            if (used_vtx.count(num)) {
                Edges e1(num, e.v1, e.v2), e2(num, e.v2, e.v1);
                if (front.act_edges.count(e1) || front.act_edges.count(e2)) {
                    if (front.act_edges.count(e1)) front.act_edges.insert(e1);
                    else front.act_edges.erase(e1);
                    if (front.act_edges.count(e2)) front.act_edges.insert(e2);
                    else front.act_edges.erase(e2);
                    triangles.push_back(num);
                    triangles.push_back(e.v1);
                    triangles.push_back(e.v2);
                    front.act_edges.erase(e);
                    return;
                }
                front.act_edges.erase(e);
            } else {
                used_vtx.insert(num);
                Edges e1(num, e.v1, e.v2), e2(num, e.v2, e.v1);
                front.act_edges.insert(e1);
                front.act_edges.insert(e2);
                triangles.push_back(num);
                triangles.push_back(e.v1);
                triangles.push_back(e.v2);
                front.act_edges.erase(e);
                return;
            }
        }
        front.act_edges.erase(e);
    }
    void mesh() {
        while (true) {
            if (! find_seed()) {
                indx += 1;
                if (indx < radii.size()) {
                    r_now = radii[indx];
                    grid.load(2.0 * r_now, pc);
                } else {
                    return;
                }
            }
            while (front.act_edges.size()) {
                auto e = *front.act_edges.rbegin();
                pivot_ball(e);
            }
            break;
        }
    }
};
void BPA_run(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh) {
    auto distances = pc.ComputeNearestNeighborDistance();

    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double              rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { rho * 2.0 };
    BPA                 bpa(pc, radii);
    bpa.mesh();
    printf("finish\n");
    mesh.Indices = bpa.triangles;
    for (auto v : pc.points_) { mesh.Positions.push_back(glm::vec3(1)); }
    for (int i = 0; i < pc.points_.size(); i++) { mesh.Positions[i] = eigen2glm(pc.points_[i]); }
}
