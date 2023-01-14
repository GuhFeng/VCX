#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "pointcloud.h"
#include "utils.h"
#include <algorithm>
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
                PMIN[i] = min(PMIN[i], v[i] - r * 2);
                PMAX[i] = max(PMAX[i], v[i] + r * 2);
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
        for (int i = 0; i < pcd.points_.size(); i++) {
            auto     v = eigen2glm(pcd.points_[i]);
            uint32_t m[3];
            for (int j = 0; j < 3; j++) m[j] = uint32_t((v - PMIN)[j] / r);
            this->lst[GET_INDX(m[0], m[1], m[2], this->n)].push_back(i);
        }
    }
    std::vector<uint32_t> Get_Neighbor(glm::vec3 v, double rad) {
        uint32_t m[3];
        if (rad > r) printf("warning: the rad is too large!\n");
        for (int i = 0; i < 3; i++) m[i] = uint32_t((v - PMIN)[i] / r);
        std::vector<uint32_t> lst_n, lst_2;
        for (int i = m[0] - 1; i < m[0] + 2; i++) {
            for (int j = m[1] - 1; j < m[1] + 2; j++) {
                for (int k = m[2] - 1; k < m[2] + 2; k++) {
                    for (int cnt = 0; cnt < this->lst[GET_INDX(i, j, k, this->n)].size(); cnt++) {
                        lst_2.push_back(this->lst[GET_INDX(i, j, k, this->n)][cnt]);
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
    uint32_t active;
    Edges() { active = 1; }
    Edges(uint32_t p1, uint32_t p2, uint32_t po) {
        v1     = p1;
        v2     = p2;
        v_op   = po;
        active = 1;
    }
    bool operator<(const Edges & e) const {
        if (v1 != e.v1) return v1 < e.v1;
        return v2 < e.v2;
    }
    bool operator==(const Edges & e) const { return (v1 == e.v1) && (v2 == e.v2); }
};

struct Front {
    std::set<Edges> act_edges;
    std::set<Edges> Boundary;
};

struct Vertex {
    uint32_t           indx;
    uint32_t           state;
    std::vector<Edges> edges;
    Vertex(uint32_t inx) {
        indx  = inx;
        state = 0;
    }
    bool operator<(const Vertex & v) const { return indx < v.indx; }
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
        center.push_back(n); // c2->v0->c1
        return 1;
    }
    return 0;
}

struct BPA {
    Front                  front;
    std::vector<uint32_t>  triangles;
    std::vector<double>    radii;
    double                 r_now;
    std::set<Edges>        used_edge;
    uint32_t               indx;
    Grid                   grid;
    PCD_t &                pc;
    std::vector<Vertex>    vtx;
    std::vector<glm::vec3> Pos;
    std::vector<glm::vec3> Nor;
    BPA(PCD_t & pcd, std::vector<double> & r): radii(r), pc(pcd) {
        r_now = r[0];
        indx  = 0;
        for (auto v : pcd.points_) Pos.push_back(eigen2glm(v));
        for (auto nor : pcd.points_) Nor.push_back(eigen2glm(nor));
        for (int i = 0; i < pcd.points_.size(); i++) vtx.push_back(Vertex(i));
        grid.load(r_now * 2.0, pcd);
    }

    bool valid_ball(glm::vec3 c, glm::vec3 nor, glm::vec3 dir) {
        auto neibors = grid.Get_Neighbor(c, r_now - 1e-5);
        if (neibors.size() == 0 && glm::dot(nor, dir) >= 0) return 1;
        return 0;
    }

    bool find_seed() {
        for (int p = 0; p < pc.points_.size(); p++) {
            glm::vec3 v = eigen2glm(pc.points_[p]), nor = eigen2glm(pc.normals_[p]);
            if (vtx[p].state) continue;
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
                    Edges e1(p, p1, p2), e2(p2, p, p1), e3(p1, p2, p);
                    triangles.push_back(p);
                    triangles.push_back(p1);
                    triangles.push_back(p2);
                    front.act_edges.insert(e1);
                    front.act_edges.insert(e2);
                    front.act_edges.insert(e3);
                    vtx[p].state  = 1;
                    vtx[p1].state = 1;
                    vtx[p2].state = 1;
                    vtx[p].edges.push_back(e1);
                    vtx[p].edges.push_back(e2);
                    vtx[p1].edges.push_back(e1);
                    vtx[p1].edges.push_back(e3);
                    vtx[p2].edges.push_back(e3);
                    vtx[p2].edges.push_back(e2);
                    return 1;
                }
            }
            vtx[p].state = 3;
        }
        return 0;
    }

    uint32_t pivot_ball(Edges e) {
        glm::vec3 mid       = eigen2glm((pc.points_[e.v1] + pc.points_[e.v2]) / 2.0);
        auto      neighbors = grid.Get_Neighbor(mid, 2.0 * r_now);
        auto      v1 = eigen2glm(pc.points_[e.v1]), v2 = eigen2glm(pc.points_[e.v2]);
        int       cnt = 0;
        for (auto nbr : neighbors) {
            if (nbr == e.v1 || nbr == e.v2 || nbr == e.v_op) continue;
            auto vk = eigen2glm(pc.points_[nbr]), nk = eigen2glm(pc.normals_[nbr]);
            std::vector<glm::vec3> centers;
            if (get_center(v1, v2, vk, r_now, centers)) {
                auto center1 = centers[0], center2 = centers[1], dir = centers[2];
                if (valid_ball(center1, nk, dir) || valid_ball(center2, nk, -dir)) { return nbr; }
            }
        }
        return -1;
    }

    void join(Edges e, uint32_t pk) {
        front.act_edges.insert(Edges(e.v1, pk, e.v2));
        front.act_edges.insert(Edges(pk, e.v2, e.v1));
        front.act_edges.erase(e);
        vtx[pk].state = 1;
    }

    void glue(Edges e1, Edges e2) {
        if (e1.v1 != e2.v2 || e1.v2 != e2.v1) printf("Warning: can't glue!\n");
        auto v1 = e1.v1, v2 = e2.v2;
        front.act_edges.erase(e1);
        front.act_edges.erase(e2);
    }

    void mesh() {
        int i = 1;
        while (true) {
            while (front.act_edges.size() > 0) {
                auto e = *front.act_edges.begin();
                if (used_edge.count(e)) {
                    front.act_edges.erase(e);
                    continue;
                } else {
                    used_edge.insert(e);
                }
                uint32_t pk = pivot_ball(e);
                if ((pk != -1) && ((vtx[pk].state == 0) || (vtx[pk].state == 1))) {
                    triangles.push_back(e.v1);
                    triangles.push_back(pk);
                    triangles.push_back(e.v2);
                    join(e, pk);
                    if (front.act_edges.count(Edges(pk, e.v1, 0))) {
                        glue(Edges(pk, e.v1, 0), Edges(e.v1, pk, 0));
                    }
                    if (front.act_edges.count(Edges(e.v2, pk, 0))) {
                        glue(Edges(pk, e.v2, 0), Edges(e.v2, pk, 0));
                    }
                } else {
                    front.act_edges.erase(e);
                    front.Boundary.insert(e);
                }
            }
            if (! find_seed()) return;
            i++;
            if (i == 3) { return; }
        }
    }
};
void BPA_run(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh) {
    auto distances = pc.ComputeNearestNeighborDistance();

    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double              rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { 2.0 * rho };
    BPA                 bpa(pc, radii);
    bpa.mesh();
    printf("finish\n");
    mesh.Indices = bpa.triangles;
    for (auto v : pc.points_) { mesh.Positions.push_back(glm::vec3(1)); }
    for (int i = 0; i < pc.points_.size(); i++) { mesh.Positions[i] = eigen2glm(pc.points_[i]); }
}