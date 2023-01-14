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

struct Edge;
struct Vertex {
    glm::vec3      Pos;
    uint32_t       indx;
    std::set<Edge> Edge;
    Vertex(int i) { indx = i; }
    Vertex() {}
    bool operator<(const Vertex & v) const { return indx < v.indx; }
    bool operator==(const Vertex & v) const { return indx == v.indx; }
};
struct Edge {
    uint32_t indx1, indx2, indx_op;
    uint32_t onfront, triangles;
    Edge(uint32_t p1, uint32_t p2, uint32_t po) {
        indx1     = max(p1, p2);
        indx2     = min(p1, p2);
        indx_op   = po;
        onfront   = 1;
        triangles = 1;
    }
    bool operator<(const Edge & e) const {
        if ((indx1 + indx2) != (e.indx1 + e.indx2)) return (indx1 + indx2) < (e.indx1 + e.indx2);
        return abs((int) (indx1 - indx2)) < abs((int) (e.indx1 - e.indx2));
    }
    bool operator==(const Edge & e) const {
        return ((indx1 == e.indx1) && (indx2 == e.indx2))
            || ((indx1 == e.indx2) && (indx2 == e.indx1));
    }
};
struct Triangle {
    uint32_t indx[3];
    Triangle(uint32_t v1, uint32_t v2, uint32_t v3) {
        indx[0] = v1;
        indx[1] = v2;
        indx[2] = v3;
        std::sort(indx, indx + 3);
    }
    bool operator==(const Triangle & t) const {
        return (indx[0] == t.indx[0]) && (indx[1] == t.indx[1]) && (indx[2] == t.indx[2]);
    }
};
struct Front {
    std::set<Edge>   active_edge;
    std::set<Vertex> acitive_vertex;
};
struct Grid {
    double                             radii;
    std::vector<glm::vec3>             posi;
    std::vector<std::vector<uint32_t>> lst;
    glm::vec3                          PMIN, PMAX;
    uint32_t                           n[3];
    void                               load(double r, std::vector<glm::vec3> & ps) {
        if (! lst.empty()) lst.clear();
        if (posi.size()) posi.clear();
        radii = r;
        PMIN  = glm::vec3(1e5);
        PMAX  = glm::vec3(-1e5);
        for (auto v : ps) {
            posi.push_back(v);
            for (int i = 0; i < 3; i++) {
                PMIN[i] = min(PMIN[i], v[i] - r * 2);
                PMAX[i] = max(PMAX[i], v[i] + r * 2);
            }
        }
        for (int i = 0; i < 3; i++) this->n[i] = uint32_t((PMAX - PMIN)[i] / r);
        for (int i = 0; i < n[0]; i++) {
            for (int j = 0; j < n[1]; j++) {
                for (int k = 0; k < n[2]; k++) { lst.push_back(std::vector<uint32_t>()); }
            }
        }
        for (int i = 0; i < ps.size(); i++) {
            auto     v = ps[i];
            uint32_t m[3];
            for (int j = 0; j < 3; j++) m[j] = uint32_t((v - PMIN)[j] / r);
            this->lst[GET_INDX(m[0], m[1], m[2], this->n)].push_back(i);
        }
    }
    std::vector<uint32_t> Get_Neighbor(glm::vec3 v, double rad) {
        uint32_t m[3];
        if (rad > radii) printf("warning: the rad is too large!\n");
        for (int i = 0; i < 3; i++) m[i] = uint32_t((v - PMIN)[i] / radii);
        std::vector<uint32_t> lst_n;
        for (int i = m[0] - 1; i < m[0] + 2; i++) {
            for (int j = m[1] - 1; j < m[1] + 2; j++) {
                for (int k = m[2] - 1; k < m[2] + 2; k++) {
                    for (int cnt = 0; cnt < lst[GET_INDX(i, j, k, this->n)].size(); cnt++) {
                        if (glm::length(v - posi[lst[GET_INDX(i, j, k, this->n)][cnt]])
                            <= rad + 1e-6)
                            lst_n.push_back(lst[GET_INDX(i, j, k, this->n)][cnt]);
                    }
                }
            }
        }
        return lst_n;
    }
};
struct BPA {
    std::vector<glm::vec3>     posi;
    std::vector<glm::vec3>     normals;
    std::map<uint32_t, Vertex> used_vertex;
    Front                      front;
    std::vector<Triangle>      triangles;
    double                     r;
    Grid                       grid;
    BPA(const VCX::Engine::SurfaceMesh & mesh, double rad) {
        posi    = mesh.Positions;
        normals = mesh.Normals;
        r       = rad;
        grid.load(2.0 * r, posi);
    }
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
        if (center.size()) center.clear();
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
    bool valid_center(glm::vec3 center, glm::vec3 dir, glm::vec3 nor) {
        auto neibors = grid.Get_Neighbor(center, r - 1e-5);
        if ((neibors.size() == 0) && (glm::dot(nor, dir) >= 0.0)) return 1;
        return 0;
    }
    bool find_seed() {
        for (int i = 0; i < posi.size(); i++) {
            if (used_vertex.count(i)) continue;
            auto neibors = grid.Get_Neighbor(posi[i], 2.0 * r);
            for (int j = 0; j < neibors.size(); j++) {
                for (int k = j + 1; k < neibors.size(); k++) {
                    auto                   p1 = neibors[j], p2 = neibors[k], p = (uint32_t) i;
                    std::vector<glm::vec3> centers;
                    if (get_center(posi[p1], posi[p2], posi[p], r, centers)) {
                        auto neibors0 = grid.Get_Neighbor(centers[0], r - 1e-5);
                        auto neibors1 = grid.Get_Neighbor(centers[1], r - 1e-5);
                        bool b1       = (neibors0.size() == 0);
                        bool b2       = (neibors1.size() == 0);
                        if (! b1 && ! b2) continue;
                        used_vertex[p1] = Vertex(p1);
                        used_vertex[p2] = Vertex(p2);
                        used_vertex[p]  = Vertex(p);
                        Edge e1(p1, p2, p), e2(p1, p, p2), e3(p, p2, p1);
                        used_vertex[p1].Edge.insert(e1);
                        used_vertex[p1].Edge.insert(e2);
                        used_vertex[p2].Edge.insert(e1);
                        used_vertex[p2].Edge.insert(e3);
                        used_vertex[p].Edge.insert(e3);
                        used_vertex[p].Edge.insert(e2);
                        front.active_edge.insert(e1);
                        front.active_edge.insert(e2);
                        front.active_edge.insert(e3);
                        triangles.push_back(Triangle(p1, p2, p));
                        return 1;
                    }
                }
            }
            used_vertex[i] = Vertex(i);
        }
        return 0;
    }
    bool pivot_ball(Edge e) {
        auto p1 = posi[e.indx1], p2 = posi[e.indx2];
        auto mid     = (p1 + p2) / glm::vec3(2.0);
        auto neibors = grid.Get_Neighbor(mid, 2.0 * r);
        for (uint32_t indx : neibors) {
            if (indx == e.indx_op || indx == e.indx1 || indx == e.indx2) continue;
            auto                   p = posi[indx];
            std::vector<glm::vec3> centers;
            if (get_center(p1, p2, p, r, centers)) {
                auto neibors0 = grid.Get_Neighbor(centers[0], r - 1e-5);
                auto neibors1 = grid.Get_Neighbor(centers[1], r - 1e-5);
                bool b1       = (neibors0.size() == 0);
                bool b2       = (neibors1.size() == 0);
                if (! b1 && ! b2) continue;
            } else {
                continue;
            }
            if (used_vertex.count(indx)) {
                Edge e1(indx, e.indx1, e.indx2), e2(indx, e.indx2, e.indx1);
                if (front.active_edge.count(e1) || front.active_edge.count(e2)) {
                    if (front.active_edge.count(e1)) {
                        front.active_edge.erase(e1);
                    } else {
                        front.active_edge.insert(e1);
                    }
                    if (front.active_edge.count(e2)) {
                        front.active_edge.erase(e2);
                    } else {
                        front.active_edge.insert(e2);
                    }
                    triangles.push_back(Triangle(e.indx1, e.indx2, indx));
                    front.active_edge.erase(e);
                    return 1;
                } else { // TODO
                }
            } else {
                used_vertex[indx] = Vertex(indx);
                e.onfront         = 0;
                e.triangles       = 2;
                Edge e1(indx, e.indx1, e.indx2), e2(indx, e.indx2, e.indx1);
                used_vertex[e.indx1].Edge.insert(e1);
                used_vertex[e.indx2].Edge.insert(e2);
                front.active_edge.insert(e1);
                front.active_edge.insert(e2);
                triangles.push_back(Triangle(e.indx1, e.indx2, indx));
                front.active_edge.erase(e);
                return 1;
            }
        }
        front.active_edge.erase(e);
        return 0;
    }
    void mesh() {
        int num = 0;
        while (true) {
            if (find_seed()) {
                num += 1;
            } else {
                break;
            }
            while (front.active_edge.size()) {
                auto e = *front.active_edge.begin();
                if (pivot_ball(e)) {
                    num += 1;
                    if (num % 1000 == 0) { printf("%d %d\n", front.active_edge.size(), num); }
                }
            }
        }
    }
};

void BPA_run(
    const VCX::Engine::SurfaceMesh & old,
    open3d::geometry::PointCloud &   pc,
    VCX::Engine::SurfaceMesh &       mesh) {
    auto   distances = pc.ComputeNearestNeighborDistance();
    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double              rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { 2.0 * rho };
    BPA *               bpa = new BPA(old, radii[0]);
    bpa->mesh();
    for (auto v : bpa->posi) { mesh.Positions.push_back(v); }
    for (auto t : bpa->triangles) {
        mesh.Indices.push_back(t.indx[0]);
        mesh.Indices.push_back(t.indx[1]);
        mesh.Indices.push_back(t.indx[2]);
    }
}