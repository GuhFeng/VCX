# __*Lab2 Report*__

## Task 1: Loop Mesh Subdivision

在这个任务中，我实现了三角网格细分算法。每一轮细分操作，在三角形的每一条边上产生一个顶点，并且对原有的顶点重新调整。因此，在实现过程中，对于每一轮细分操作，我分成了四步：
* 建立查询边的数据结构
* 计算原有的顶点的位置
* 计算新生成的顶点位置
* 建立顶点之间的连接关系
在第一步中，我使用助教提供的`DCEL`数据结构来查询边。在第二步中，遍历每一个顶点，根据周围顶点的位置和该定点原来的位置来计算这个顶点的新位置,并且按照周围顶点权重$\frac{3}{8n}$，该顶点权重$\frac{5}{8}$进行加权。在第三步中，遍历每一条边，根据论文中的公式进行加权计算出新加入的顶点的位置，并且记录每一边所加入的顶点的索引。在这里，我定义了一个宏。第四步中，遍历每一个三角形，根据之前保存的索引，将新的四个三角形加入到新的mesh中，在这里需要注意加入点的顺序。
```C++
// Macro
#define MAP_PAIR(a, b) ((((uint64_t) a + b) << 32) + abs((long long) (a -  b)))
// Map the edge (v2,v3) to the index of new vertex
map_record[MAP_PAIR(v2, v3)] = New.Positions.size() - 1;
```

## Task 2: Spring-Mass Mesh Parameterization

在这个任务中，我实现了基于弹簧质点的三角网格参数化算法。首先，我使用助教提供的`DCEL`数据结构来查询边。然后，通过循环，我找到了一个边界点，以这个边界点为起点，按照顺序找出所有的边界点。而后，我将边界点上的$(u,v)$坐标初始化为圆边界。最后，我使用迭代法求解中间节点上的$(u,v)$坐标，为了方便，我直接使用$\lambda_{ij}=1/n_i$ 的平均权重。
```C++
\\ Init side vertex
for (int i = 0; i < side_vertex.size(); i++) {
        output.TexCoords[side_vertex[i]] = glm::vec2(
            0.5 + 0.5 * sin(2 * PI * i / side_vertex.size()),
            0.5 + 0.5 * cos(2 * PI * i / side_vertex.size()));
} 
\\ Iteration
texco.push_back(glm::vec2(0));
for (int j = 0; j < v_neighbors.size(); j++) {
    uint32_t u = v_neighbors[j];
    texco[i] = texco[i] + glm::vec2(1.0 / v_neighbors.size()) * output.TexCoords[u];
}
```
