#include "vtk_reader.h"

#include <fstream>
#include <iostream>
#include <vector>

namespace io {
int read_mesh_from_vtk(const std::string &file_name, Eigen::Matrix3Xd &V,
                       Eigen::MatrixXi &T, int mesh_type) {
    std::ifstream ifs(file_name);
    if (ifs.fail()) {
        std::cerr << "[info] "
                  << "can not open file" << file_name << std::endl;
        return __LINE__;
    }

    std::string str;
    size_t point_num = 0, cell_num = 0;
    while (!ifs.eof()) {
        ifs >> str;
        if (str == "POINTS") {
            ifs >> point_num >> str;
            V.resize(3, point_num);
            for (size_t i = 0; i < point_num; ++i) {
                for (size_t j = 0; j < 3; ++j)
                    ifs >> V(j, i);
            }
        } else if (str == "CELLS") {
            const size_t num_nods = mesh_type == 1 ? 4 : 8;
            ifs >> cell_num >> str;
            size_t point_number_of_cell = 0;
            std::vector<int> tet_temp;
            for (size_t ci = 0; ci < cell_num; ++ci) {
                ifs >> point_number_of_cell;
                if (point_number_of_cell != num_nods) {
                    for (size_t i = 0; i < point_number_of_cell; ++i)
                        ifs >> str;
                } else {
                    int p;
                    for (size_t i = 0; i < point_number_of_cell; ++i) {
                        ifs >> p;
                        tet_temp.push_back(p);
                    }
                }
            }
            T = Eigen::Map<const Eigen::MatrixXi>(tet_temp.data(), num_nods,
                                                  cell_num);
        }
    }
    return 0;
}

int read_mesh_from_vtk(const std::string &file_name, Eigen::Matrix3Xd &V,
                       Eigen::Matrix4Xi &T, Eigen::MatrixXi &P,
                       Eigen::MatrixXi &H) {
    std::ifstream ifs(file_name);
    if (ifs.fail()) {
        std::cerr << "[info] "
                  << "can not open file" << file_name << std::endl;
        return __LINE__;
    }

    std::string str;
    while (!ifs.eof()) {
        ifs >> str;
        if (str == "POINTS") {
            size_t point_num;
            ifs >> point_num >> str;
            V.resize(3, point_num);
            for (size_t i = 0; i < point_num; ++i) {
                for (size_t j = 0; j < 3; ++j)
                    ifs >> V(j, i);
            }
        } else if (str == "CELLS") {
            size_t cell_num, tmp;
            ifs >> cell_num >> tmp;
            size_t point_number_of_cell = 0;
            std::vector<int> tet, pag, hex;
            for (size_t ci = 0; ci < cell_num; ++ci) {
                ifs >> point_number_of_cell;
                if (point_number_of_cell == 4) {
                    for (size_t i = 0; i < point_number_of_cell; ++i) {
                        ifs >> tmp;
                        tet.push_back(tmp);
                    }
                } else if (8 == point_number_of_cell) {
                    for (size_t i = 0; i < point_number_of_cell; ++i) {
                        ifs >> tmp;
                        hex.push_back(tmp);
                    }
                } else if (5 == point_number_of_cell) {
                    for (size_t i = 0; i < point_number_of_cell; ++i) {
                        ifs >> tmp;
                        pag.push_back(tmp);
                    }
                }
            }
            if (tet.size() > 0)
                T = Eigen::Map<const Eigen::Matrix4Xi>(tet.data(), 4,
                                                       tet.size() / 4);
            if (pag.size() > 0)
                P = Eigen::Map<const Eigen::MatrixXi>(pag.data(), 5,
                                                      pag.size() / 5);
            if (hex.size() > 0) {
                H = Eigen::Map<const Eigen::MatrixXi>(hex.data(), 8,
                                                      hex.size() / 8);
            }
            assert(T.cols() + H.cols() + P.cols() == cell_num);
        }
    }
    return 0;
}

} // namespace io
