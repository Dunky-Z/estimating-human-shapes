#pragma once

#include <string>
#include <Eigen/Dense>



namespace  meshio {
// @brief read and write obj mesh
// @param filename: the file name of the mesh
// @param V: the vertex list of the mesh
// @param F: the face list of the mesh
// @return 0: read or write failed; 1: read or write successufully

int ReadObj(const std::string &filename, Eigen::Matrix3Xd &vertex, Eigen::Matrix3Xi &facets);
int ReadObj(const std::string &filename, Eigen::MatrixXd &vertex);
int SaveObj(const std::string &filename, const Eigen::Matrix3Xd &nods,const Eigen::Matrix3Xi &tris);
}

