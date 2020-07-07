#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

#include "pmp/SurfaceMesh.h"

using namespace pmp;
using namespace std;
using namespace Eigen;

#define RIGHT_TURN -1  // CW
#define LEFT_TURN 1  // CCW
#define COLLINEAR 0  // Collinear


const string ori_mesh_path = "./model/male01.obj";
const string points_path = "./model/points.txt";
const string convex_hull_path = "./model/convexhull.txt";

const std::string DATASET_PATH = "./model/";
//const std::string DATASET_PATH = "./model/";
const std::string BIN_DATA_PATH = "./model/";
//const std::string BIN_DATA_PATH = "./model/";

/*!
*@brief  将点集写入文件
*@param[out]
*@param[in]  const std::string & filename
*@param[in]  const std::vector<pmp::vec3> & points
*@return     int
*/
//int  SavePointToFile(const std::string &filename, const std::vector<pmp::vec3>& points)
//{
//	std::ofstream os(filename.c_str());
//	if (!os)
//		return 0;
//	for (auto& p : points)
//	{
//		os << p[0] << " " << p[1] << "\n";
//	}
//	os.close();
//	return 1;
//}
