#include <glm/glm.hpp>
#include <Open3D/Open3D.h>

int main() {
    open3d::geometry::PointCloud pd;
    open3d::io::ReadPointCloud("bunny.xyz", pd);
}