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

#define VERTS 12500
#define FACES 25000
#define EPS 1e-6
#define BASIS_NUM 10
#define EPSILON 1e-6


const string ori_mesh_path = "./data/male01.obj";
const string points_path = "./data/points.txt";
const string convex_hull_path = "./data/convexhull.txt";

const std::string DATASET_PATH = "D:/ITabc/ITabc/objDataSet/dataset/male_tmp/";
//const std::string DATASET_PATH = "./model/";
const std::string BIN_DATA_PATH = "./data/";
//const std::string BIN_DATA_PATH = "./model/";

std::vector<std::string> GetFiles(const std::string & cate_dir);