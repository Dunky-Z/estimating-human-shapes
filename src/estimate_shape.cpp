#include "utils.h"
#include "estimate_shape.h"

void Estimate::Apply()
{
	Estimate estimate;
	SurfaceMesh mesh;
	mesh.read(ori_mesh_path);
	cout << mesh.n_vertices() << endl;

	estimate.SetBool(mesh);
}
//
//void Estimate::SetBool(SurfaceMesh&mesh)
//{
//	auto is_checked = mesh.add_edge_property<bool>("e:isChecked");
//	for (auto e : mesh.edges())
//	{
//		is_checked[e] = 0;
//	}
//}
//
//float Estimate::CheckIntersection(pmp::vec3 intersect_point, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, const SurfaceMesh & mesh)
//{
//	pmp::vec3 p0 = mesh.position(mesh.vertex(edg, 0));
//	pmp::vec3 p1 = mesh.position(mesh.vertex(edg, 1));
//	pmp::vec3 p01 = p1 - p0;
//	float res = pmp::dot(normal, p01);
//	//res=0˵������ƽ��ƽ��
//	if (res == 0)
//	{
//		return -1;
//	}
//	//�������ӣ������������λ��
//	float scale = (dot(normal, (v - p0))) / res;
//	CalcIntersectionPoint(intersect_point, mesh, scale, p0, p1);
//
//	return scale;
//}
//
//void Estimate::CalcIntersectionPoint(pmp::vec3 intersect_point, const SurfaceMesh& mesh, const float scale, const pmp::vec3& p0, const pmp::vec3& p1)
//{
//
//	//����ཻ�ڶ��㣬���㼴��������
//	if (scale == 0)
//	{
//		intersect_point = p0;
//	}
//	else
//	{
//		intersect_point = scale * (p1 - p0) + p0;
//	}
//}
//
//float Estimate::CalcChainLength(std::vector<pmp::vec3> intersect_points)
//{
//	float chain_len = 0;
//	int n = intersect_points.size();
//	//��Ϊֻ��Ϊ������㼯����ܳ��������ų���С�������ܳ�
//	//����û�м������һ�������һ����֮��ľ���
//	for (int i = 1; i < n; ++i)
//	{
//		int len = distance(intersect_points[i - 1], intersect_points[i]);
//		chain_len += len;
//	}
//	return chain_len;
//}
//
//void Estimate::CalcIntersectionPoints(std::vector<pmp::vec3> intersect_points, std::vector<pmp::Edge> edgs_intersect, std::vector<float> scale_set,
//	SurfaceMesh& mesh, const pmp::vec3& normal, const pmp::vec3& v)
//{
//	std::vector<pmp::Edge> edgs_intersect;
//	std::vector<float> scale_set;
//	pmp::vec3 intersect_point;
//	//����������㼯���ֱ�Ϊ�м������Χ�����ֱ۵�Χ��
//	std::vector<pmp::vec3> intersect_points_t, intersect_points;
//	int chain_len = INT_MIN, chain_len_t;
//
//	auto is_checked = mesh.get_edge_property<bool>("e:isChecked");
//	for (auto &edg : mesh.edges())
//	{
//		//�����Ѿ������ı�
//		if (is_checked[edg]) continue;
//
//		float scale = CheckIntersection(intersect_point, edg, normal, v, mesh);
//		if (scale != -1)
//		{
//			is_checked[edg] = 1;
//			scale_set.push_back(scale);
//			edgs_intersect.push_back(edg);
//			intersect_points_t.push_back(intersect_point);
//			FindIntersectionEdgeNearby(is_checked, scale_set, intersect_points_t, edgs_intersect, mesh, edg, normal, v, intersect_point);
//			//�����ܳ�����Ǹ��㼯
//			chain_len_t = CalcChainLength(intersect_points_t);
//			if (chain_len_t > chain_len)
//			{
//				chain_len = chain_len_t;
//				intersect_points = intersect_points_t;
//			}
//		}
//	}
//}
//
//
//
//void Estimate::FindIntersectionEdgeNearby(pmp::EdgeProperty<bool> is_checked, std::vector<float> scale_set, std::vector<pmp::vec3> intersect_points,
//	std::vector<pmp::Edge> edgs_intersect, const SurfaceMesh& mesh, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, pmp::vec3 intersect_point)
//{
//	pmp::Halfedge half_edge = mesh.halfedge(edg, 0);
//	pmp::Halfedge half_edge_next = mesh.next_halfedge(half_edge);
//	while (half_edge != half_edge_next)
//	{
//		pmp::Edge edg_next = mesh.edge(half_edge_next);
//		float scale = CheckIntersection(intersect_point, edg, normal, v, mesh);
//		if (scale != -1)
//		{
//			is_checked[edg_next] = 1;
//			scale_set.push_back(scale);
//			edgs_intersect.push_back(edg_next);
//			intersect_points.push_back(intersect_point);
//			half_edge_next = mesh.opposite_halfedge(half_edge_next);
//			if (half_edge_next == half_edge)
//			{
//				break;
//			}
//		}
//		half_edge_next = mesh.next_halfedge(half_edge_next);
//	}
//}