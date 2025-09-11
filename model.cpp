#include <fstream>
#include <sstream>
#include <iostream>
#include "model.h"

Model::Model(const std::string filename) {
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Eigen::Vector3f v;
            for (int i : {0,1,2}) iss >> v[i];
            verts.push_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            int f,t,n, cnt = 0;
            iss >> trash;
            while (iss >> f >> trash >> t >> trash >> n) {
                facet_vrt.push_back(--f);
                cnt++;
            }
            if (3!=cnt) {
                std::cerr << "Error: the obj file is supposed to be triangulated" << std::endl;
                return;
            }
        } else if (!line.compare(0, 2, "vn ")) {
            Eigen::Vector3f v;
            for (int i : {0,1,2}) iss >> v[i];
            vertex_normals.push_back(v);
        } else if (!line.compare(0, 2, "vt ")) {
            Eigen::Vector3f v;
            for (int i : {0,1,2}) iss >> v[i];
            vertex_texture_coordinate.push_back(v);
        }
    }
    std::cerr << "# v# " << nverts() << " f# "  << nfaces() << std::endl;
}

int Model::nverts() const { return verts.size(); }
int Model::nfaces() const { return facet_vrt.size()/3; }

Eigen::Vector3f Model::vert(const int i) const {
    return verts[i];
}

Eigen::Vector3f Model::vert(const int iface, const int nthvert) const {
    return verts[facet_vrt[iface*3+nthvert]];
}