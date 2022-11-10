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
    uint32_t ** allocate(int a, int b, int type = 0) {
        uint32_t ** p = new uint32_t *[a];
        for (int i = 0; i < a; i++) {
            if (type) {
                p[i] = new uint32_t[b];
                memset(p[i], -1, sizeof(uint32_t) * b);
            } else {
                p[i] = (uint32_t *) new double[b];
                memset(p[i], 0, sizeof(uint64_t) * b);
            }
        }
        return p;
    }

    void unalloc(int a, uint32_t ** p, int type = 1) {
        if (type) {
            for (int i = 0; i < a; i++) delete[] p[i];
        } else {
            for (int i = 0; i < a; i++) delete[] (double *) p[i];
        }
        delete[] p;
    }

    uint32_t *** allocate_3d(int a, int b, int c) {
        uint32_t *** p = new uint32_t **[a];
        for (int i = 0; i < a; i++) {
            p[i] = new uint32_t *[b];
            for (int j = 0; j < b; j++) {
                p[i][j] = new uint32_t[c];
                memset(p[i][j], 0, sizeof(uint32_t) * c);
            }
        }
        return p;
    }

    void unalloc_3d(int a, int b, uint32_t *** p) {
        for (int i = 0; i < a; i++) {
            delete[] p[i];
            for (int j = 0; j < b; j++) delete[] p[i][j];
        }
        delete[] p;
    }

    float my_cot(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
        glm::vec3 d1  = v1 - v3;
        glm::vec3 d2  = v2 - v3;
        double    COS = ((d1 * d2)[0] + (d1 * d2)[1] + (d1 * d2)[2]);
        COS           = COS / (pow(((d1 * d1)[0] + (d1 * d1)[1] + (d1 * d1)[2]), 0.5));
        COS           = COS / (pow(((d2 * d2)[0] + (d2 * d2)[1] + (d2 * d2)[2]), 0.5));
        /*if (COS >= 1) {
            printf("wrong!%f\n", COS);
            printf("%f %f %f   %f %f %f\n", d1[0], d1[1], d1[2], d2[0], d2[1], d2[2]);
            printf(
                "%f    %f \n",
                ((d1 * d2)[0] + (d1 * d2)[1] + (d1 * d2)[2]),
                (powf(((d1 * d1)[0] + (d1 * d1)[1] + (d1 * d1)[2]), 0.5))
                    * powf(((d2 * d2)[0] + (d2 * d2)[1] + (d2 * d2)[2]), 0.5));
        }*/
        if (COS >= 1 - 1e-12) return COS = 1 - 1e-12;
        return (COS / sqrt(1 - COS * COS)) / 1e3;
    }

    void show(const Engine::SurfaceMesh & mesh) {
        printf("Positions: %ld \n", mesh.Positions.size());
        for (glm::vec3 p : mesh.Positions) printf("%f %f %f   ", p.p, p.y, p.z);
        printf("\nTriangles: %ld \n", (mesh.Indices.size() / 3));
        for (int i = 0; i < mesh.Indices.size(); i += 3)
            printf("%d %d %d   ", mesh.Indices[i], mesh.Indices[i + 1], mesh.Indices[i + 2]);
        printf("\n");
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
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(
        Engine::SurfaceMesh const & input,
        Engine::SurfaceMesh &       output,
        float                       valid_pair_threshold,
        float                       simplification_ratio) {
        // your code here
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
            double ** w = (double **) allocate(Old.Positions.size(), Old.Positions.size(), 0);
            for (DCEL::HalfEdge const * e : links.GetEdges()) {
                std::uint32_t v0 = e->OppositeVertex();
                std::uint32_t v1 = e->PairOppositeVertex();
                std::uint32_t v2 = e->From();
                std::uint32_t v3 = e->To();
                if (useUniformWeight) {
                    w[v2][v3] = 1.0;
                    w[v3][v2] = 1.0;
                } else {
                    w[v2][v3] = my_cot(Old.Positions[v2], Old.Positions[v3], Old.Positions[v1]);
                    w[v2][v3] =
                        w[v2][v3] + my_cot(Old.Positions[v2], Old.Positions[v3], Old.Positions[v0]);
                    w[v3][v2] = w[v2][v3];
                }
            }
            for (std::size_t i = 0; i < Old.Positions.size(); ++i) {
                DCEL::Vertex                        v             = links.GetVertex(i);
                std::vector<const DCEL::Triangle *> faces         = v.GetFaces();
                std::vector<std::uint32_t>          neighbor_list = v.GetNeighbors();
                glm::vec3                           pos(0.0);
                float                               W = 0;
                for (int j = 0; j < neighbor_list.size(); j++) {
                    pos = pos + Old.Positions[neighbor_list[j]] * glm::vec3(w[i][neighbor_list[j]]);
                    W += w[i][neighbor_list[j]];
                }
                pos = pos / glm::vec3(W);
                pos = pos * glm::vec3(lambda) + Old.Positions[i] * glm::vec3(1 - lambda);
                New.Positions.push_back(pos);
            }
            unalloc(Old.Positions.size(), (uint32_t **) w, 0);
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
    }
} // namespace VCX::Labs::GeometryProcessing
