////#include <iostream>
//#include <vector>
//#include <Eigen/Dense>
//#include <unordered_map>
//#include "pmp/SurfaceMesh.h"
//#include <pmp/algorithms/SurfaceNormals.h>
//#include <pmp/algorithms/DifferentialGeometry.h>
//
//using namespace Eigen;
//using namespace pmp;
//using namespace std;
//using Point = pmp::Point;
//
//
//const string ori_obj = "D:/systemFolder/Desktop/pic/david.obj";
//const string nosise_obj = "D:/systemFolder/Desktop/pic/david_L0.obj";
////const string ori_obj = "D:/systemFolder/Desktop/pic/fandisk.obj";
////const string nosise_obj = "D:/systemFolder/Desktop/pic/fandisk_no.obj";
//
//int main()
//{
//	unordered_map<float, int> mp_freq;//频数
//	unordered_map<float, float> mp_prob;//概率
//	SurfaceMesh mesh_ori;
//	SurfaceMesh mesh_denoise;
//	mesh_ori.read(ori_obj);
//	mesh_denoise.read(nosise_obj);
//	SurfaceNormals::compute_face_normals(mesh_ori);
//	SurfaceNormals::compute_face_normals(mesh_denoise);
//
//	auto ori_fnormals = mesh_ori.get_face_property<Normal>("f:normal");
//	auto de_fnormals = mesh_denoise.get_face_property<Normal>("f:normal");
//
//	//统计每个角度差出现的次数
//	float angles = 0.0;
//	for (auto f : mesh_ori.faces())
//	{
//		auto fn1 = ori_fnormals[f];
//		auto fn2 = de_fnormals[f];
//		float angle_ = floor(angle(fn1, fn2) * 1000) / 1000.0;
//		mp_freq[angle_]++;
//	}
//
//	//角度差出现的概率
//	int num_faces = mesh_ori.n_faces();
//	auto it = mp_freq.begin();
//	while (it != mp_freq.end())
//	{
//		mp_prob[it->first] = it->second / (float)num_faces;
//		it++;
//	}
//
//	//求期望值
//	auto it_prob = mp_prob.begin();
//	float expt = 0.0;
//	while (it_prob != mp_prob.end())
//	{
//		expt += (it_prob->first * (it_prob->second));
//		it_prob++;
//	}
//	cout << expt << endl;
//	return 0;
//}
