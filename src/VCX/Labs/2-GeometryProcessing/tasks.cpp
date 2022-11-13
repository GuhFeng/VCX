#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#define MAP_PAIR(a, b) ((uint64_t) (a + b) << 32) + abs((long long) ((long) a - (long) b))

#define INDEX_CUBE(a, b, c, n) (a - 1) * n * n + b(b - 1) * n + c - 1

#define VERTEX_I(p, i) \
    glm::vec3((p[0] + (i & 1) * dx), p[1] + ((i >> 1) & 1) * dx, p[2] + ((i >> 2) & 1) * dx)

    float my_cot(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
        glm::vec3 d1  = v1 - v3;
        glm::vec3 d2  = v2 - v3;
        float     COS = ((d1 * d2)[0] + (d1 * d2)[1] + (d1 * d2)[2])
            * ((d1 * d2)[0] + (d1 * d2)[1] + (d1 * d2)[2]);
        COS = COS / ((d1 * d1)[0] + (d1 * d1)[1] + (d1 * d1)[2]);
        COS = COS / ((d2 * d2)[0] + (d2 * d2)[1] + (d2 * d2)[2]);
        if (COS >= 1 - 1e-12) return COS = 1 - 1e-12;
        if (COS <= -1 + 1e-12) return COS = -1 + 1e-12;
        return sqrt(COS / sqrt(1 - COS));
    }

    void show(const Engine::SurfaceMesh & mesh) {
        printf("Positions: %ld \n", mesh.Positions.size());
        for (glm::vec3 p : mesh.Positions) printf("%f %f %f   ", p.p, p.y, p.z);
        printf("\nTriangles: %ld \n", (mesh.Indices.size() / 3));
        for (int i = 0; i < mesh.Indices.size(); i += 3)
            printf("%d %d %d   ", mesh.Indices[i], mesh.Indices[i + 1], mesh.Indices[i + 2]);
        printf("\n");
    }

    glm::vec4 plane_equition(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
        glm::vec3 a = v1 - v2, b = v1 - v3;
        a = glm::vec3(
            a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
        b = a * a;
        if (b[0] + b[1] + b[2] == 0) return glm::vec4(1, 0, 0, 0);
        a = a / glm::vec3(sqrt(b[0] + b[1] + b[2]));
        b = a * v1;
        return glm::vec4(a[0], a[1], a[2], -b[0] - b[1] - b[2]);
    }

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(
        Engine::SurfaceMesh const & input,
        Engine::SurfaceMesh &       output,
        std::uint32_t               numIterations) {
        output = input;
        for (int cnt = 0; cnt < numIterations; cnt++) {
            Engine::SurfaceMesh & Old   = *(new Engine::SurfaceMesh(output));
            Engine::SurfaceMesh & New   = *(new Engine::SurfaceMesh);
            DCEL &                links = *(new DCEL);
            links.AddFaces(Old.Indices);
            if (! links.IsValid()) printf("Invalid Mesh\n");
            for (std::size_t i = 0; i < Old.Positions.size(); ++i) {
                DCEL::Vertex               v             = links.GetVertex(i);
                std::vector<std::uint32_t> neighbor_list = v.GetNeighbors();
                glm::vec3                  pos(0.0);
                for (std::uint32_t indx : neighbor_list) { pos = pos + Old.Positions[indx]; }
                pos = pos * (glm::vec3(0.375 / neighbor_list.size()));
                pos = pos + Old.Positions[i] * (glm::vec3(0.625));
                New.Positions.push_back(pos);
            }
            std::map<uint64_t, uint32_t> map_record;
            for (DCEL::HalfEdge const * e : links.GetEdges()) {
                std::uint32_t v0 = e->OppositeVertex();
                std::uint32_t v1 = e->PairOppositeVertex();
                std::uint32_t v2 = e->From();
                std::uint32_t v3 = e->To();
                glm::vec3     pos(0);
                pos = pos + (Old.Positions[v0] + Old.Positions[v1]) * (glm::vec3(0.125));
                pos = pos + (Old.Positions[v2] + Old.Positions[v3]) * (glm::vec3(0.375));
                New.Positions.push_back(pos);
                map_record[((uint64_t) (v2) << 32) + v3] = New.Positions.size() - 1;
                map_record[((uint64_t) (v3) << 32) + v2] = New.Positions.size() - 1;
            }
            for (int i = 0; i < Old.Indices.size(); i = i + 3) {
                const std::uint32_t * f  = Old.Indices.data() + i;
                uint32_t              v1 = f[0], v2 = f[1], v3 = f[2];
                uint32_t              v4 = map_record[((uint64_t) (v2) << 32) + v1];
                uint32_t              v5 = map_record[((uint64_t) (v3) << 32) + v1];
                uint32_t              v6 = map_record[((uint64_t) (v3) << 32) + v2];
                New.Indices.push_back(v1);
                New.Indices.push_back(v4);
                New.Indices.push_back(v5);
                New.Indices.push_back(v4);
                New.Indices.push_back(v2);
                New.Indices.push_back(v6);
                New.Indices.push_back(v3);
                New.Indices.push_back(v5);
                New.Indices.push_back(v6);
                New.Indices.push_back(v6);
                New.Indices.push_back(v5);
                New.Indices.push_back(v4);
            }
            output = New;
            delete &Old;
            delete &New;
            delete &links;
        }
    }
    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(
        Engine::SurfaceMesh const & input,
        Engine::SurfaceMesh &       output,
        const std::uint32_t         numIterations) {
        // your code here
        DCEL & links = *new DCEL;
        links.AddFaces(input.Indices);
        output = input;
        std::vector<uint32_t> side_vertex;
        uint32_t              v_start = 0, v_prev = 0, v_now = 0;
        for (std::size_t i = 0; i < output.Positions.size(); ++i) {
            DCEL::Vertex v = links.GetVertex(i);
            if (v.IsSide()) {
                v_start = i;
                v_prev  = i;
                v_now   = v.GetSideNeighbors().first;
                side_vertex.push_back(v_prev);
                break;
            }
        }
        while (v_now != v_start) {
            DCEL::Vertex v  = links.GetVertex(v_now);
            uint32_t     v1 = v.GetSideNeighbors().first, v2 = v.GetSideNeighbors().second;
            side_vertex.push_back(v_now);
            if (v1 == v_prev) {
                v_prev = v_now;
                v_now  = v2;
            } else {
                v_prev = v_now;
                v_now  = v1;
            }
        }
        double PI = acos(-1);
        for (int i = 0; i < output.Positions.size(); i++) {
            output.TexCoords.push_back(glm::vec2(0.5 + 0 * i / output.Positions.size()));
        }
        for (int i = 0; i < side_vertex.size(); i++) {
            output.TexCoords[side_vertex[i]] = glm::vec2(
                0.5 + 0.5 * sin(2 * PI * i / side_vertex.size()),
                0.5 + 0.5 * cos(2 * PI * i / side_vertex.size()));
        }
        for (int cnt_iter = 0; cnt_iter < numIterations; cnt_iter++) {
            std::vector<glm::vec2> texco;
            for (int i = 0; i < output.Positions.size(); i++) {
                DCEL::Vertex v = links.GetVertex(i);
                if (v.IsSide()) {
                    texco.push_back(output.TexCoords[i]);
                    continue;
                }
                std::vector<uint32_t> v_neighbors = v.GetNeighbors();
                std::vector<float>    Dis;
                float                 D_sum = 0;
                for (uint32_t u : v_neighbors) {
                    glm::lowp_vec2 d = output.TexCoords[i] - output.TexCoords[u];
                    d                = d * d;
                    Dis.push_back(1);
                    D_sum = D_sum + 1;
                }
                texco.push_back(glm::vec2(0));
                for (int j = 0; j < Dis.size(); j++) {
                    uint32_t u = v_neighbors[j];
                    texco[i]   = texco[i] + glm::vec2(Dis[j] / D_sum) * output.TexCoords[u];
                }
            }
            output.TexCoords = texco;
        }
        delete &links;
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(
        Engine::SurfaceMesh const & input,
        Engine::SurfaceMesh &       output,
        float                       valid_pair_threshold,
        float                       simplification_ratio) {
        // your code here
        output.Positions = input.Positions;
        DCEL & links     = *new DCEL;
        links.AddFaces(input.Indices);
        std::vector<glm::fmat4x4> Qs;
        for (int i = 0; i < input.Positions.size(); i++) {
            DCEL::Vertex                        v     = links.GetVertex(i);
            std::vector<const DCEL::Triangle *> faces = v.GetFaces();
            std::vector<glm::vec4>              ps;
            for (const DCEL::Triangle * f : faces) {
                uint32_t v_indx[3];
                for (int cnt = 0; cnt < 3; cnt++) v_indx[cnt] = *f->Indices(cnt);
                ps.push_back(plane_equition(
                    input.Positions[v_indx[0]],
                    input.Positions[v_indx[1]],
                    input.Positions[v_indx[2]]));
            }
            glm::fmat4x4 Q(0.0);
            for (glm::vec4 p : ps) {
                for (int a = 0; a < 4; a++)
                    for (int b = 0; b < 4; b++) Q[a][b] = Q[a][b] + p[a] * p[b];
            }
            Qs.push_back(Q);
        }
        std::set<uint64_t>     records;
        std::vector<float>     costs;
        std::vector<uint64_t>  valid_pair_1;
        std::vector<uint64_t>  valid_pair_2;
        std::vector<glm::vec4> vertexes;
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            std::uint32_t v1 = e->From();
            std::uint32_t v2 = e->To();
            valid_pair_1.push_back(v1);
            valid_pair_2.push_back(v2);
            records.insert(MAP_PAIR(v1, v2));
        }
        for (int i = 0; i < input.Positions.size(); i++) {
            for (int j = 0; j < input.Positions.size(); j++) {
                uint64_t record = *records.upper_bound(MAP_PAIR(i, j));
                if (record == MAP_PAIR(i, j)) continue;
                else {
                    glm::vec3 dist = (input.Positions[i] - input.Positions[j])
                        * (input.Positions[i] - input.Positions[j]);
                    float d = dist[0] + dist[1] + dist[2];
                    if (d < valid_pair_threshold) {
                        valid_pair_1.push_back(i);
                        valid_pair_2.push_back(j);
                        records.insert(MAP_PAIR(i, j));
                    }
                }
            }
        }
        for (int i = 0; i < valid_pair_1.size(); i++) {
            uint32_t     v1 = valid_pair_1[i], v2 = valid_pair_2[i];
            glm::fmat4x4 Q = Qs[v1] + Qs[v2];
            glm::fmat4x4 Q_new(
                Q[0][0],
                Q[0][1],
                Q[0][2],
                0,
                Q[1][0],
                Q[1][1],
                Q[1][2],
                0,
                Q[2][0],
                Q[2][1],
                Q[2][2],
                0,
                Q[3][0],
                Q[3][1],
                Q[3][2],
                1);
            glm::vec4 v_new;
            if (glm::determinant(Q) <= 0) {
                glm::vec3 a = input.Positions[v1] + input.Positions[v2];
                a           = a / glm::vec3(2);
                v_new       = glm::vec4(a[0], a[1], a[2], 1);
            } else {
                v_new = glm::inverse(Q_new) * glm::vec4(0, 0, 0, 1);
            }
            vertexes.push_back(v_new);
            glm::vec4 tmp = Q * v_new;
            costs.push_back(
                tmp[0] * v_new[0] + tmp[1] * v_new[1] + tmp[2] * v_new[2] + tmp[3] * v_new[3]);
        }
        int                   dcnt = 0;
        int                   dbg  = 100;
        std::vector<uint32_t> v_map;
        size_t                numIter = (1 - simplification_ratio) * input.Positions.size();
        for (int i = 0; i < input.Positions.size(); i++) v_map.push_back(-1);
        for (int iter = 0; iter < numIter; iter++) {
            float tmp = 1000000;
            int   cnt = 0;
            for (int i = 0; i < valid_pair_1.size(); i++) {
                uint32_t v1 = valid_pair_1[i], v2 = valid_pair_2[i];
                if (tmp > costs[i]) {
                    cnt = i;
                    tmp = costs[i];
                }
            }
            uint32_t v1 = valid_pair_1[cnt], v2 = valid_pair_2[cnt];
            uint32_t tmp1 = v1, tmp2 = v2;
            costs[cnt] = 100000;
            glm::vec3 new_pos(vertexes[cnt][0], vertexes[cnt][1], vertexes[cnt][2]);
            output.Positions.push_back(new_pos);
            v_map.push_back(-1);
            uint32_t v3 = output.Positions.size() - 1;
            while (v_map[v1] != -1) v1 = v_map[v1];
            while (v_map[v2] != -1) v2 = v_map[v2];
            v_map[v1]   = v3;
            v_map[v2]   = v3;
            v_map[tmp1] = v3;
            v_map[tmp2] = v3;
            dcnt++;
            if (dcnt == dbg) break;
        }
        for (int i = 0; i < input.Indices.size(); i = i + 3) {
            uint32_t v1 = input.Indices[i];
            uint32_t v2 = input.Indices[i + 1];
            uint32_t v3 = input.Indices[i + 2];
            while (v_map[v1] != -1) v1 = v_map[v1];
            while (v_map[v2] != -1) v2 = v_map[v2];
            while (v_map[v3] != -1) v3 = v_map[v3];
            if (v1 == v2 or v1 == v3 or v2 == v3) continue;
            else {
                output.Indices.push_back(v1);
                output.Indices.push_back(v2);
                output.Indices.push_back(v3);
            }
        }
        delete &links;
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(
        Engine::SurfaceMesh const & input,
        Engine::SurfaceMesh &       output,
        std::uint32_t               numIterations,
        float                       lambda,
        bool                        useUniformWeight) {
        // your code here
        output = input;
        for (int cnt = 0; cnt < numIterations; cnt++) {
            Engine::SurfaceMesh & Old   = *(new Engine::SurfaceMesh(output));
            Engine::SurfaceMesh & New   = *(new Engine::SurfaceMesh);
            DCEL &                links = *(new DCEL);
            links.AddFaces(Old.Indices);
            if (! links.IsValid()) printf("Invalid Mesh\n");
            std::map<uint64_t, double> map_w;
            for (DCEL::HalfEdge const * e : links.GetEdges()) {
                std::uint32_t v0 = e->OppositeVertex();
                std::uint32_t v1 = e->PairOppositeVertex();
                std::uint32_t v2 = e->From();
                std::uint32_t v3 = e->To();
                if (useUniformWeight) {
                    map_w[MAP_PAIR(v2, v3)] = 1.0;
                } else {
                    map_w[MAP_PAIR(v2, v3)] =
                        my_cot(Old.Positions[v2], Old.Positions[v3], Old.Positions[v1])
                        + my_cot(Old.Positions[v2], Old.Positions[v3], Old.Positions[v0]);
                }
            }
            for (std::size_t i = 0; i < Old.Positions.size(); ++i) {
                DCEL::Vertex                        v             = links.GetVertex(i);
                std::vector<const DCEL::Triangle *> faces         = v.GetFaces();
                std::vector<std::uint32_t>          neighbor_list = v.GetNeighbors();
                glm::vec3                           pos(0.0);
                float                               W = 0;
                for (int j = 0; j < neighbor_list.size(); j++) {
                    uint32_t v2 = i, v3 = neighbor_list[j];
                    pos =
                        pos + Old.Positions[neighbor_list[j]] * glm::vec3(map_w[MAP_PAIR(v2, v3)]);
                    W += map_w[MAP_PAIR(v2, v3)];
                }
                pos = pos / glm::vec3(W);
                pos = pos * glm::vec3(lambda) + Old.Positions[i] * glm::vec3(1 - lambda);
                New.Positions.push_back(pos);
            }
            for (int i = 0; i < Old.Indices.size(); i++) New.Indices.push_back(Old.Indices[i]);
            output = New;
            delete &Old;
            delete &New;
            delete &links;
        }
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(
        Engine::SurfaceMesh &                           output,
        const std::function<float(const glm::vec3 &)> & sdf,
        const glm::vec3 &                               grid_min,
        const float                                     dx,
        const int                                       n) {
        // your code here
        std::vector<uint32_t>        cubes;
        std::map<uint64_t, uint32_t> indxs;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < n; k++) {
                    uint32_t  sgns  = 0;
                    glm::vec3 pos_0 = grid_min + glm::vec3(i * dx, j * dx, k * dx);
                    for (int a = 0; a < 8; a++) sgns += (sdf(VERTEX_I(pos_0, i)) > 0) << a;
                }
            }
        }
    }
} // namespace VCX::Labs::GeometryProcessing
