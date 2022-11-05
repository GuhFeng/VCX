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

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    void Something(Engine::SurfaceMesh const & input) {
        DCEL links;
        links.AddFaces(input.Indices); // initialize
        if (! links.IsValid()) {
            // we check if the mesh is valid
            printf("Invalid Mesh");
        }

        // for each vertex
        for (std::size_t i = 0; i < input.Positions.size(); ++i) {
            DCEL::Vertex v = links.GetVertex(i); // get vertex with index i
            // do something with v
        }

        // for each edge
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            // do something with e
        }
    }

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(
        Engine::SurfaceMesh const & input,
        Engine::SurfaceMesh &       output,
        std::uint32_t               numIterations) {
        // your code here
        DCEL       links;
        uint32_t * new_vertex_indx = new uint32_t[links.GetEdges().size()];
        uint32_t   cnt             = 0;
        if (! links.IsValid()) printf("Invalid Mesh\n");
        for (std::size_t i = 0; i < input.Positions.size(); ++i) {
            DCEL::Vertex               v             = links.GetVertex(i);
            std::vector<std::uint32_t> neighbor_list = v.GetNeighbors();
            glm::vec3                  pos;
            for (std::uint32_t indx : neighbor_list) { pos = pos + input.Positions[indx]; }
            pos = pos * (glm::vec3(0.375 * neighbor_list.size()));
            pos = pos + input.Positions[i] * (glm::vec3(0.625 * neighbor_list.size()));
            output.Positions.push_back(pos);
        }
        char * is_processed = (char *) malloc(links.GetFaces().size());
        memset(is_processed, 0, links.GetFaces().size());
        cnt = 0;
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            std::uint32_t v0 = e->OppositeVertex();
            std::uint32_t v1 = e->PairOppositeVertex();
            std::uint32_t v2 = e->From();
            std::uint32_t v3 = e->To();
            glm::vec3     pos;
            pos = pos + (input.Positions[v0] + input.Positions[v1]) * (glm::vec3(0.125));
            pos = pos + (input.Positions[v2] + input.Positions[v3]) * (glm::vec3(0.375));
            output.Positions.push_back(pos);
            new_vertex_indx[cnt] = output.Positions.size() - 1;
        }

        output = input;
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
