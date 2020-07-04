#pragma once
#include "pmp/SurfaceMesh.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace pmp;
using namespace std;

class Estimate
{
public:
	Estimate();
	~Estimate();
	void Apply();
	float CheckIntersection(pmp::vec3 intersect_point, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, const SurfaceMesh & mesh);
	void CalcIntersectionPoint(pmp::vec3& intersect_point, const float& scale, const SurfaceMesh& mesh, const pmp::vec3& p0, const pmp::vec3& p1);

	void SetBool(SurfaceMesh&mesh);
	float CalcChainLength(std::vector<pmp::vec3>& intersect_points);
	void CalcIntersectionPoints(std::vector<pmp::vec3>& intersect_points, std::vector<pmp::Edge>& edgs_intersect, std::vector<float>& scale_set,
		SurfaceMesh& mesh, const pmp::vec3& normal, const pmp::vec3& v);

	void FindIntersectionEdgeNearby(pmp::EdgeProperty<bool>& is_checked, std::vector<float>& scale_set, std::vector<pmp::vec3>& intersect_points,
		std::vector<pmp::Edge>& edgs_intersect, const SurfaceMesh& mesh, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, pmp::vec3 intersect_point);
private:
	const pmp::vec3 normal = vec3(0, 0, 1);
	const pmp::vec3 ori_point = vec3(0, 0, 0);
};
