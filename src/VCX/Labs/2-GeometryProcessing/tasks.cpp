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

    uint32_t ** allocate(int a, int b) {
        uint32_t ** p = new uint32_t *[a];
        for (int i = 0; i < a; i++) {
            p[i] = new uint32_t[b];
            memset(p[i], -1, sizeof(uint32_t) * b);
        }
        return p;
    }
    void unalloc(int a, uint32_t ** p) {
        for (int i = 0; i < a; i++) delete[] p[i];
        delete[] p;
    }
    void show(const Engine::SurfaceMesh & mesh) {
        printf("Positions: %d \n", mesh.Positions.size());
        for (glm::vec3 p : mesh.Positions) printf("%f %f %f   ", p.p, p.y, p.z);
        printf("\nTriangles: %d \n", (mesh.Indices.size() / 3));
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
        // your code here
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
            uint32_t ** record = allocate(Old.Positions.size(), Old.Positions.size());
            for (DCEL::HalfEdge const * e : links.GetEdges()) {
                std::uint32_t v0 = e->OppositeVertex();
                std::uint32_t v1 = e->PairOppositeVertex();
                std::uint32_t v2 = e->From();
                std::uint32_t v3 = e->To();
                glm::vec3     pos(0);
                pos = pos + (Old.Positions[v0] + Old.Positions[v1]) * (glm::vec3(0.125));
                pos = pos + (Old.Positions[v2] + Old.Positions[v3]) * (glm::vec3(0.375));
                New.Positions.push_back(pos);
                record[v2][v3] = New.Positions.size() - 1;
                record[v3][v2] = New.Positions.size() - 1;
            }
            for (int i = 0; i < Old.Indices.size(); i = i + 3) {
                const std::uint32_t * f = Old.Indices.data() + i;
                New.Indices.push_back(f[0]);
                New.Indices.push_back(record[f[0]][f[1]]);
                New.Indices.push_back(record[f[0]][f[2]]);
                New.Indices.push_back(record[f[0]][f[1]]);
                New.Indices.push_back(f[1]);
                New.Indices.push_back(record[f[2]][f[1]]);
                New.Indices.push_back(f[2]);
                New.Indices.push_back(record[f[0]][f[2]]);
                New.Indices.push_back(record[f[1]][f[2]]);
                New.Indices.push_back(record[f[1]][f[2]]);
                New.Indices.push_back(record[f[0]][f[2]]);
                New.Indices.push_back(record[f[0]][f[1]]);
            }
            unalloc(Old.Positions.size(), record);

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
