//
// Created by 黄凯成 on 2025/9/11.
//

#ifndef RENDER_MODEL_H
#define RENDER_MODEL_H


#include <vector>
#include <Eigen/Eigen>

class Model {
    std::vector<Eigen::Vector3f> verts = {};    // array of vertices
    std::vector<Eigen::Vector3f> vertex_normals = {};    // array of vertices
    std::vector<Eigen::Vector3f> vertex_texture_coordinate = {};    // array of vertices
    std::vector<int> facet_vrt = {}; // per-triangle index in the above array
public:
    Model(const std::string filename);
    int nverts() const; // number of vertices
    int nfaces() const; // number of triangles
    Eigen::Vector3f vert(const int i) const;                          // 0 <= i < nverts()
    Eigen::Vector3f vert(const int iface, const int nthvert) const;   // 0 <= iface <= nfaces(), 0 <= nthvert < 3


};


#endif //RENDER_MODEL_H