#pragma once
#include <string>
#include<fstream>
#include<iostream>
#include <Eigen/Dense>

#include "pmp/SurfaceMesh.h"

using namespace pmp;
using namespace std;
using namespace Eigen;
using namespace std;

#define RIGHT_TURN -1  // CW
#define LEFT_TURN 1  // CCW
#define COLLINEAR 0  // Collinear


const pmp::vec3 normal = vec3(0, 0, 1);
const pmp::vec3 ori_point = vec3(0, 0, 0);

const string ori_mesh_path = "./model/male01.obj";
const string filepath = "./model/points.txt";


/*!
*@brief  将点集写入文件
*@param[out]
*@param[in]  const std::string & filename
*@param[in]  const std::vector<pmp::vec3> & points
*@return     int
*/
int  SavePointToFile(const std::string &filename, const std::vector<pmp::vec3>& points)
{
	std::ofstream os(filename.c_str());
	if (!os)
		return 0;
	for (auto& p : points)
	{
		os << p[0] << " " << p[1] << "\n";
	}
	os.close();
	return 1;
}

/*!
*@brief  通过向量叉积判断方向
*@param[out] 
*@param[in]  const pmp::vec3 & p  
*@param[in]  const pmp::vec3 & q  
*@param[in]  const pmp::vec3 & r  
*@return     float  
*/
float orientation(const pmp::vec3& p, const pmp::vec3& q, const pmp::vec3& r)
{
	//float val = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
	float val = (q[1] - p[1]) * (r[0] - p[0]) - (q[0] - p[0]) * (r[1] - p[1]);
	if (val == 0) return 0;  // Collinear
	return (val > 0) ? -1 : 1; // CW: -1 or CCW: 1
}


/*!
*@brief  排序函数辅助函数
*@param[out] 
*@param[in]  const pmp::vec3 & p1  
*@param[in]  const pmp::vec3 & p2  
*@return     bool  
*/
bool compare(const pmp::vec3& p1, const pmp::vec3& p2)
{
	int orient = orientation(ori_point, p1, p2);
	if (orient == 0)
		return ((distance(ori_point, p2) >= distance(ori_point, p1)) ? 0 : 1);
	return (orient == 1) ? 0 : 1;
	//return (orient > 0) || (orient == 0 && (distance(ori_point, p2) >= distance(ori_point, p1)));
}


/*!
*@brief  Toleft检测
*@param[out] 
*@param[in]  vector<pmp::vec3> & v  
*@param[in]  pmp::vec3 & p  
*@return     std::vector<pmp::vec3>  
*/
vector<pmp::vec3>keep_left(vector<pmp::vec3>& v, pmp::vec3& p)
{
	while (v.size() > 1 && orientation(v[v.size() - 2], v[v.size() - 1], p) != RIGHT_TURN)
		v.pop_back();
	if (!v.size() || v[v.size() - 1] != p)
		v.push_back(p);
	return v;
}


/*!
*@brief  
*@param[out] 
*@param[in]  vector<pmp::vec3> & points  
*@return     std::vector<pmp::vec3>  
*/
vector<pmp::vec3>GrahamScan(vector<pmp::vec3>& points)
{
	if (points.size() <= 1)
		return points;
	sort(points.begin(), points.end(), compare);
	vector<pmp::vec3> lower_hull;
	for (int i = 0; i < points.size(); ++i)
		lower_hull = keep_left(lower_hull, points[i]);
	reverse(points.begin(), points.end());
	vector<pmp::vec3> upper_hull;
	for (int i = 0; i < points.size(); ++i)
		upper_hull = keep_left(upper_hull, points[i]);
	for (int i = 1; i < upper_hull.size(); ++i)
		lower_hull.push_back(upper_hull[i]);
	return lower_hull;
}
