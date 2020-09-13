#pragma once

#include <algorithm>
#include <Eigen/Dense>

#include "utils.h"
#include "binary_io.h"
#include "estimate_shape.h"

using namespace std;
class Measure
{
private:
	static const int M_NUM = 15;
	const std::vector<int> cir_index = { 8791  ,7233 ,5333 };//4-Chest,5-Belly button waist,6-Gluteal hip

public:

	Measure();
	~Measure();

	double CalcVariance(const Eigen::MatrixXd &x, const double average);
	double CalcStd(const Eigen::MatrixXd &x, const double average);
	Eigen::MatrixXd Measure::CalcMeasure(const std::vector<std::vector<std::vector<double>>>& control_points, const Eigen::Matrix3Xd & vertices,
		const Eigen::Matrix3Xi &facets, Eigen::MatrixX3f& circum, int index);
	Eigen::MatrixXd Measure::CalcMeasure(const std::vector<std::vector<std::vector<double>>>& control_points, const Eigen::Matrix3Xd & vertices,
		const Eigen::Matrix3Xi &facets);
	void Measure::ConvertMeasure(const Eigen::MatrixXd & all_vertices, const Eigen::Matrix3Xi &facets,
		const std::vector<std::vector<std::vector<double>>>& control_points, Eigen::MatrixXd &measure_lists, Eigen::MatrixX3f& circum);
	float orientation(const pmp::vec3& p, const pmp::vec3& q, const pmp::vec3& r);
	bool compare(const pmp::vec3& p1, const pmp::vec3& p2);
	vector<pmp::vec3> keep_left(vector<pmp::vec3>& v, pmp::vec3& p);
	vector<pmp::vec3>GrahamScan(vector<pmp::vec3>& points);

	float CheckIntersection(pmp::vec3& intersect_point, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, const SurfaceMesh & mesh);
	void CalcIntersectionPoint(pmp::vec3& intersect_point, const float& scale, const SurfaceMesh& mesh, const pmp::vec3& p0, const pmp::vec3& p1);

	void SetBool(SurfaceMesh&mesh);
	float CalcChainLength(std::vector<pmp::vec3>& intersect_points);
	void CalcIntersectionPoints(std::vector<pmp::vec3>& intersect_points, std::vector<pmp::Edge>& edgs_intersect, std::vector<float>& scale_set,
		SurfaceMesh& mesh, const pmp::vec3& normal, const pmp::vec3& v);

	void FindIntersectionEdgeNearby(pmp::EdgeProperty<bool>& is_checked, std::vector<float>& scale_set, std::vector<pmp::vec3>& intersect_points,
		std::vector<pmp::Edge>& edgs_intersect, SurfaceMesh& mesh, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, pmp::vec3 intersect_point);



	float Measure::CalcConvexCircumference(SurfaceMesh& mesh, const int index);
	std::vector<float> Measure::CalcConvexCircumferences(SurfaceMesh& mesh);
	void Measure::CalcCircumferencesAndSave();
public:
	const pmp::vec3 normal = vec3(0, 0, 1);
	

};