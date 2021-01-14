#pragma once

#include <io.h>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <fstream>

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

struct node {
	int x, y;
	double t;
};

const string ori_mesh_path = "./data/male01.obj";
const string points_path = "./data/points.txt";
const string points_idx_path = "./data/points_id.txt";
const string convex_hull_path = "./data/convexhull.txt";

const std::string DATASET_PATH = "D:/ITabc/ITabc/objDataSet/dataset/female_tmp/";
//const std::string DATASET_PATH = "./model/";
const std::string BIN_DATA_PATH = "./data/";
//const std::string BIN_DATA_PATH = "./model/";
int  SavePointToFile(const std::string &filename, const std::vector<pmp::vec3>& points);
int  SavePointToFile(const std::string &filename, const std::vector<std::vector<int>>& point_idx);
std::vector<std::string> GetFiles(const std::string & cate_dir);