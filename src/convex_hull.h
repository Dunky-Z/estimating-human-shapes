#pragma once
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <algorithm>	// For qsort() algorithm
#include <utility>		// For pair() STL

#include "pmp/SurfaceMesh.h"
#include <Eigen/Dense>
#include <functional>


#include "utils.h"
#include "estimate_shape.h"

#define RIGHT_TURN -1  // CW
#define LEFT_TURN 1  // CCW
#define COLLINEAR 0  // Collinear

using namespace std;
using namespace pmp;
using namespace Eigen;
using std::placeholders::_1;
using std::placeholders::_2;


class ConvexHull
{
public:
	ConvexHull();
	~ConvexHull();
	int orientation(pmp::vec3 p, pmp::vec3 q, pmp::vec3 r);
	int compare(const pmp::vec3 p1, const pmp::vec3 p2);
	int tangent(vector<pmp::vec3> v, pmp::vec3 p);
	pair<int, int> extreme_hullpt_pair(vector<vector<pmp::vec3> >& hulls);
	pair<int, int> next_hullpt_pair(vector<vector<pmp::vec3> >& hulls, pair<int, int> lpoint);
	vector<pmp::vec3> chansalgorithm(vector<pmp::vec3> v);
	vector<pmp::vec3> GrahamScan(vector<pmp::vec3>& points);
	vector<pmp::vec3> keep_left(vector<pmp::vec3>& v, pmp::vec3 p);

private:
	const pmp::vec3 normal = vec3(0, 0, 1);
	const pmp::vec3 ori_point = vec3(0, 0, 0);
};
