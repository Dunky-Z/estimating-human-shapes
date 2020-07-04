#include "utils.h"
#include "estimate_shape.h"


Estimate::Estimate()
{
}

Estimate::~Estimate()
{
}


void Estimate::Apply()
{
	SurfaceMesh mesh;
	mesh.read(ori_mesh_path);

	Estimate estimate;
	std::vector<float> scale_set;
	std::vector<pmp::Edge> edgs_intersect;
	std::vector<pmp::vec3> intersect_points;
	cout << mesh.n_vertices() << endl;

	estimate.SetBool(mesh);
	estimate.CalcIntersectionPoints(intersect_points, edgs_intersect, scale_set, mesh, normal, ori_point);

}

/*!
*@brief  每条边设置是否被访问标记
*@param[out]
*@param[in]  SurfaceMesh & mesh
*@return     void
*/void Estimate::SetBool(SurfaceMesh&mesh)
{
	auto is_checked = mesh.add_edge_property<bool>("e:isChecked");
	for (auto e : mesh.edges())
	{
		is_checked[e] = 0;
	}
}


void Estimate::CalcIntersectionPoint(pmp::vec3& intersect_point, const float& scale, const SurfaceMesh& mesh, const pmp::vec3& p0, const pmp::vec3& p1)
{

	//如果相交于顶点，交点即顶点坐标
	if (scale == 0)
	{
		intersect_point = p0;
	}
	else
	{
		intersect_point = scale * (p1 - p0) + p0;
	}
}

float Estimate::CheckIntersection(pmp::vec3 intersect_point, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, const SurfaceMesh & mesh)
{
	pmp::vec3 p0 = mesh.position(mesh.vertex(edg, 0));

	//cout << "p0: " << p0 << endl;

	pmp::vec3 p1 = mesh.position(mesh.vertex(edg, 1));
	pmp::vec3 p01 = p1 - p0;
	float res = pmp::dot(normal, p01);
	//res=0说明边与平面平行
	if (res == 0)
	{
		return -1;
	}
	//缩放因子，可以求出交点位置
	float scale = pmp::dot(normal, (v - p0)) / res;

	//cout << scale << endl;

	CalcIntersectionPoint(intersect_point, scale, mesh, p0, p1);

	return scale;
}


float Estimate::CalcChainLength(std::vector<pmp::vec3>& intersect_points)
{
	float chain_len = 0;
	int n = intersect_points.size();
	//因为只是为了求出点集大概周长，方便排除较小的两个周长
	//所以没有加上最后一个点与第一个点之间的距离
	for (int i = 1; i < n; ++i)
	{
		float len = distance(intersect_points[i - 1], intersect_points[i]);
		chain_len += len;
	}

	cout << "chain_len: " << chain_len << endl;
	return chain_len;
}

void Estimate::CalcIntersectionPoints(std::vector<pmp::vec3>& intersect_points, std::vector<pmp::Edge>& edgs_intersect, std::vector<float>& scale_set,
	SurfaceMesh& mesh, const pmp::vec3& normal, const pmp::vec3& v)
{
	pmp::vec3 intersect_point;
	//会求出三个点集，分别为中间身体的围长和手臂的围长,用一个临时数组保存其他两个点集
	std::vector<pmp::vec3> intersect_points_t;
	int chain_len = INT_MIN, chain_len_t;

	auto is_checked = mesh.get_edge_property<bool>("e:isChecked");
	for (auto &edg : mesh.edges())
	{
		//跳过已经遍历的边
		if (is_checked[edg]) continue;

		float scale = CheckIntersection(intersect_point, edg, normal, v, mesh);
		if (scale >= 0 && scale <= 1)
		{
			cout << "scale: " << scale << endl;

			is_checked[edg] = 1;
			scale_set.push_back(scale);
			edgs_intersect.push_back(edg);
			intersect_points_t.push_back(intersect_point);
			FindIntersectionEdgeNearby(is_checked, scale_set, intersect_points_t, edgs_intersect, mesh, edg, normal, v, intersect_point);

			cout << "inter point size: " << intersect_points_t.size() << endl;


			//留下周长最长的那个点集
			chain_len_t = CalcChainLength(intersect_points_t);
			cout << "chain_len: " << chain_len_t << endl;

			if (chain_len_t > chain_len)
			{
				chain_len = chain_len_t;
				intersect_points = intersect_points_t;
			}
		}
	}
}



void Estimate::FindIntersectionEdgeNearby(pmp::EdgeProperty<bool>& is_checked, std::vector<float>& scale_set, std::vector<pmp::vec3>& intersect_points,
	std::vector<pmp::Edge>& edgs_intersect, const SurfaceMesh& mesh, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, pmp::vec3 intersect_point)
{
	pmp::Halfedge half_edge = mesh.halfedge(edg, 0);
	pmp::Halfedge half_edge_next = mesh.next_halfedge(half_edge);
	int i = 0;
	while (half_edge != half_edge_next)
	{
		i++;
		pmp::Edge edg_next = mesh.edge(half_edge_next);
		float scale = CheckIntersection(intersect_point, edg, normal, v, mesh);
		if (scale >= 0 && scale <= 1)
		{
			is_checked[edg_next] = 1;
			scale_set.push_back(scale);
			edgs_intersect.push_back(edg_next);
			intersect_points.push_back(intersect_point);
			half_edge_next = mesh.opposite_halfedge(half_edge_next);

			//当某条半边的对边是起始的那条半边，就退出循环
			if (half_edge_next == half_edge)
			{
				break;
			}
		}
		half_edge_next = mesh.next_halfedge(half_edge_next);
	}
	cout << "找到了：" << i << "个边" << endl;
}