#pragma once

#include <io.h>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>


#include "pmp/SurfaceMesh.h"

using namespace pmp;
using namespace std;
using namespace Eigen;

#define RIGHT_TURN -1  // CW
#define LEFT_TURN 1  // CCW
#define COLLINEAR 0  // Collinear


const string ori_mesh_path = "./data/male01.obj";
const string points_path = "./data/points.txt";
const string convex_hull_path = "./data/convexhull.txt";

const std::string DATASET_PATH = "D:/ITabc/ITabc/objDataSet/dataset/male_tmp/";
//const std::string DATASET_PATH = "./model/";
const std::string BIN_DATA_PATH = "./data/";
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

/*!
*@brief  读取文件夹下所有文件
*@param[out]
*@param[in]  const std::string & cate_dir  文件夹路径
*@return     std::vector<std::string>  文件夹下所有文件
*/
