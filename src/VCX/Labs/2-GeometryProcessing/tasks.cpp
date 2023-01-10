#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/Ponit-Cloud/pointcloud.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

namespace VCX::Labs::GeometryProcessing {

#define MAP_PAIR(a, b) \
    (((uint64_t) ((long long) a + b) << 32) + abs((long long) ((long) a - (long) b)))

#define NODE_MAP(a, b, c, t) \
    ((a + ((t >> 0) & 1)) * 100 * 100 * 4 + (b + ((t >> 1) & 1)) * 100 * 2 + c + ((t >> 2) & 1))

#define VERTEX_POSI(p, i) \
    (glm::vec3((p[0] + (i & 1) * dx), p[1] + ((i >> 1) & 1) * dx, p[2] + ((i >> 2) & 1) * dx))

#define EDGE_MAP(a, b, c, i, j) (MAP_PAIR(NODE_MAP(a, b, c, i), NODE_MAP(a, b, c, j)))

    int cube_edges[12][2] = {
        {0, 1},
        {2, 3},
        {4, 5},
        {6, 7},
        {0, 2},
        {4, 6},
        {1, 3},
        {5, 7},
        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7}
    };

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
                map_record[MAP_PAIR(v2, v3)] = New.Positions.size() - 1;
            }
            for (int i = 0; i < Old.Indices.size(); i = i + 3) {
                const std::uint32_t * f  = Old.Indices.data() + i;
                uint32_t              v1 = f[0], v2 = f[1], v3 = f[2];
                uint32_t              v4 = map_record[MAP_PAIR(v2, v1)];
                uint32_t              v5 = map_record[MAP_PAIR(v1, v3)];
                uint32_t              v6 = map_record[MAP_PAIR(v2, v3)];
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
                texco.push_back(glm::vec2(0));
                for (int j = 0; j < v_neighbors.size(); j++) {
                    uint32_t u = v_neighbors[j];
                    texco[i] = texco[i] + glm::vec2(1.0 / v_neighbors.size()) * output.TexCoords[u];
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
        output.Indices   = input.Indices;
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
        std::set<uint64_t>           records;
        std::map<uint64_t, uint32_t> pair_map;
        std::vector<uint32_t>        valid;
        std::vector<float>           costs;
        std::vector<uint64_t>        valid_pair_1;
        std::vector<uint64_t>        valid_pair_2;
        std::vector<glm::vec4>       vertexes;
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            std::uint32_t v1 = e->From();
            std::uint32_t v2 = e->To();
            valid_pair_1.push_back(v1);
            valid_pair_2.push_back(v2);
            records.insert(MAP_PAIR(v1, v2));
            pair_map[MAP_PAIR(v1, v2)] = valid_pair_1.size() - 1;
        }
        uint32_t edge_pair_num = valid_pair_1.size();
        for (int i = 0; i < input.Positions.size(); i++) {
            for (int j = i; j < input.Positions.size(); j++) {
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
                        pair_map[MAP_PAIR(i, j)] = valid_pair_1.size() - 1;
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
            v_new = glm::inverse(Q_new) * glm::vec4(0, 0, 0, 1);
            vertexes.push_back(v_new);
            glm::vec4 tmp = Q * v_new;
            costs.push_back(
                tmp[0] * v_new[0] + tmp[1] * v_new[1] + tmp[2] * v_new[2] + tmp[3] * v_new[3]);
            valid.push_back(1);
        }
        size_t iter_num = (1 - simplification_ratio) * input.Positions.size(),
               pair_num = valid_pair_1.size();
        for (int iter_cnt = 0; iter_cnt <= iter_num; iter_cnt++) {
            uint32_t indx_del = 0;
            float    min_cost = INFINITY;
            for (uint32_t i = 0; i < pair_num; i++) {
                if (! valid[i]) continue;
                if (min_cost > costs[i]) {
                    indx_del = i;
                    min_cost = costs[i];
                }
            }
            valid[indx_del]                          = 0;
            output.Positions[valid_pair_1[indx_del]] = vertexes[indx_del];
            for (int i = 0; i < output.Indices.size(); i++) {
                if (output.Indices[i] == valid_pair_2[indx_del]) {
                    output.Indices[i] = valid_pair_1[indx_del];
                }
            }
            for (int i = 0; i < output.Indices.size(); i = i + 3) {
                if (output.Indices[i] == output.Indices[i + 1]
                    || output.Indices[i + 2] == output.Indices[i + 1]
                    || output.Indices[i + 2] == output.Indices[i]) {
                    output.Indices.erase(
                        output.Indices.begin() + i, output.Indices.begin() + i + 3);
                    i = i - 3;
                }
            }
            for (int i = 0; i < pair_num; i++) {
                if (! valid[i]) continue;
                if (valid_pair_1[i] == valid_pair_2[indx_del])
                    valid_pair_1[i] = valid_pair_1[indx_del];
                if (valid_pair_2[i] == valid_pair_2[indx_del])
                    valid_pair_2[i] = valid_pair_1[indx_del];
                if (valid_pair_1[i] == valid_pair_2[i]) valid[i] = 0;
            }
            Qs[valid_pair_1[indx_del]] = Qs[valid_pair_1[indx_del]] + Qs[valid_pair_2[indx_del]];
            for (int i = 0; i < pair_num; i++) {
                if (! valid[i]) continue;
                uint32_t v1 = valid_pair_1[i], v2 = valid_pair_2[i];
                if ((v1 != valid_pair_1[indx_del] && v1 != valid_pair_2[indx_del])
                    && (v2 != valid_pair_1[indx_del] && v2 != valid_pair_2[indx_del]))
                    continue;
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
                v_new         = glm::inverse(Q_new) * glm::vec4(0, 0, 0, 1);
                vertexes[i]   = v_new;
                glm::vec4 tmp = Q * v_new;
                costs[i] =
                    tmp[0] * v_new[0] + tmp[1] * v_new[1] + tmp[2] * v_new[2] + tmp[3] * v_new[3];
            }
        }
        std::set<uint32_t> cnt;
        for (int i = 0; i < output.Indices.size(); i++) { cnt.insert(output.Indices[i]); }
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
        std::map<uint64_t, uint32_t> indxs;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < n; k++) {
                    uint32_t  sgns  = 0;
                    glm::vec3 pos_0 = grid_min + glm::vec3(i * dx, j * dx, k * dx);
                    for (int a = 0; a < 8; a++) {
                        sgns = sgns + ((sdf(VERTEX_POSI(pos_0, a)) > 0) << a);
                    }
                    for (int a = 0; a < 12; a++) {
                        int   v1 = cube_edges[a][0], v2 = cube_edges[a][1];
                        float d1 = sdf(VERTEX_POSI(pos_0, v1)), d2 = sdf(VERTEX_POSI(pos_0, v2));
                        if (d1 * d2 < 0) {
                            d1 = -d1;
                            if (! indxs.count(EDGE_MAP(i, j, k, v1, v2))) {
                                indxs[EDGE_MAP(i, j, k, v1, v2)] = output.Positions.size();
                                output.Positions.push_back(
                                    VERTEX_POSI(pos_0, v1) * glm::vec3(abs(d2 / (d1 + d2)))
                                    + VERTEX_POSI(pos_0, v2) * glm::vec3(abs(d1 / (d1 + d2))));
                            }
                        }
                    }
                    uint32_t            e_sgns = c_EdgeStateTable[sgns];
                    std::array<int, 16> e_link = c_EdgeOrdsTable[sgns];
                    for (int a = 0; e_link[a] != -1; a = a + 3) {
                        int e1 = e_link[a], e2 = e_link[a + 1], e3 = e_link[a + 2], v1, v2;
                        v1 = cube_edges[e1][0];
                        v2 = cube_edges[e1][1];
                        output.Indices.push_back(indxs[EDGE_MAP(i, j, k, v1, v2)]);
                        v1 = cube_edges[e2][0];
                        v2 = cube_edges[e2][1];
                        output.Indices.push_back(indxs[EDGE_MAP(i, j, k, v1, v2)]);
                        v1 = cube_edges[e3][0];
                        v2 = cube_edges[e3][1];
                        output.Indices.push_back(indxs[EDGE_MAP(i, j, k, v1, v2)]);
                    }
                }
            }
        }
    }
    /******************* 6. Show Obj *****************/
    void
        PointCloud(const Engine::SurfaceMesh & old, Engine::SurfaceMesh & mesh, const char * path) {
        solve(mesh, path);
        // mesh = old;
        mesh.NormalizePositions();
    }
} // namespace VCX::Labs::GeometryProcessing