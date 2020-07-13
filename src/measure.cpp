
#include <iostream>

#include "measure.h"

using namespace std;


Measure::Measure() { }

Measure::~Measure() { }


/*!
*@brief  计算一个模型的所有尺寸
*@param[out]
*@param[in]  const std::vector<std::vector<std::vector<double>>> & control_points  控制点信息
*@param[in]  const Eigen::Matrix3Xd & vertices  一个模型的所有顶点信息
*@param[in]  const Eigen::Matrix3Xi & facets  三角面片的信息
*@return     Eigen::MatrixXd
*/
Eigen::MatrixXd Measure::CalcMeasure(const std::vector<std::vector<std::vector<double>>>& control_points, const Eigen::Matrix3Xd & vertices,
	const Eigen::Matrix3Xi &facets, Eigen::MatrixX3f& circum, int index)
{

	//cout << "start calculate measure..." << endl;
	//for (int i = 0; i < control_points.size(); ++i)
	//{
	//	for (int j = 0; j < control_points[i].size(); ++j)
	//	{
	//		for (int k = 0; k < control_points[i][j].size(); ++k)
	//		{
	//			cout << control_points[i][j][k] << "  ";
	//		}
	//		cout << endl;

	//	}
	//}

	Eigen::MatrixXd measure_list;
	measure_list.resize(1, M_NUM);
	double vol = 0.0;
	double kHumanbodyIntensity = 1026.0;
	Eigen::Vector3d v0, v1, v2;
	//cout << F.cols() << endl;
	for (int i = 0; i < facets.cols(); ++i)
	{
		v0 = vertices.col(facets(0, i));
		v1 = vertices.col(facets(1, i));
		v2 = vertices.col(facets(2, i));
		//calc area
		vol += v0.cross(v1).dot(v2);
	}
	vol = abs(vol) / 6.0;


	double weight = kHumanbodyIntensity * vol;
	weight = pow(weight, 1.0 / 3.0) * 1000;
	measure_list(0) = weight;
	//int idx = 1;
	int idx = 0, t = 0;

	//measures[i][j]  第i行数据， j列数据为顶点序号
	for (auto & measures : control_points)
	{
		//用凸包计算出来的围长替换之前的测地距离
		//if (idx == 4 || idx == 5 || idx == 6)
		//{
		//	float t = circum.coeff(index, idx - 4) * 1000;
		//	measure_list(idx++) = t;
		//	continue;
		//}
		if (t == 2 || t == 3 || t == 4)
		{	
			t++;
			continue;
		}
		t++;
		double length = 0.0;
		Eigen::Vector3d p1, p2;
		p2 = vertices.col(measures[0][1]);

		for (int i = 0; i < measures.size(); ++i)
		{
			p1 = p2;
			if (measures[i][0] == 1)
				p2 = vertices.col(measures[i][1]);
			else if (measures[i][0] == 2)
			{
				p2 = vertices.col(measures[i][1])*measures[i][3] + vertices.col(measures[i][2])*measures[i][4];
			}
			else
			{
				p2 = vertices.col(measures[i][1])*measures[i][4] + vertices.col(measures[i][2])*measures[i][5] + vertices.col(measures[i][3])*measures[i][6];
			}
			length += sqrt((p1 - p2).array().pow(2).sum());
		}
		measure_list(idx++) = length * 1000;
	}
	//shape : 19,1
	return measure_list.transpose();

	//cout << measure_list.cols() << endl;;
}

Eigen::MatrixXd Measure::CalcMeasure(const std::vector<std::vector<std::vector<double>>>& control_points, const Eigen::Matrix3Xd & vertices,
	const Eigen::Matrix3Xi &facets)
{

	//cout << "start calculate measure..." << endl;
	//for (int i = 0; i < control_points.size(); ++i)
	//{
	//	for (int j = 0; j < control_points[i].size(); ++j)
	//	{
	//		for (int k = 0; k < control_points[i][j].size(); ++k)
	//		{
	//			cout << control_points[i][j][k] << "  ";
	//		}
	//		cout << endl;

	//	}
	//}

	Eigen::MatrixXd measure_list;
	measure_list.resize(1, M_NUM);

	int idx = 0, t = 0;
	//measures[i][j]  第i行数据， j列数据为顶点序号
	for (auto & measures : control_points)
	{
		if (t == 2 || t == 3 || t == 4)
		{
			t++;
			continue;
		}
		t++;
		double length = 0.0;
		Eigen::Vector3d p1, p2;
		p2 = vertices.col(measures[0][1]);

		for (int i = 0; i < measures.size(); ++i)
		{
			p1 = p2;
			if (measures[i][0] == 1)
				p2 = vertices.col(measures[i][1]);
			else if (measures[i][0] == 2)
			{
				p2 = vertices.col(measures[i][1])*measures[i][3] + vertices.col(measures[i][2])*measures[i][4];
			}
			else
			{
				p2 = vertices.col(measures[i][1])*measures[i][4] + vertices.col(measures[i][2])*measures[i][5] + vertices.col(measures[i][3])*measures[i][6];
			}
			length += sqrt((p1 - p2).array().pow(2).sum());
		}
		measure_list(idx++) = length * 1000;
	}
	//shape : 19,1
	return measure_list.transpose();

	//cout << measure_list.cols() << endl;;
}
/*!
*@brief  计算方差
*@param[out]
*@param[in]  const Eigen::MatrixXd & x
*@param[in]  const double average
*@return     double
*/double Measure::CalcVariance(const Eigen::MatrixXd &x, const double average)
{
	double sum = 0.0;
	int len = x.cols();
	for (int i = 0; i < len; ++i)
	{
		sum += pow(x(0, i) - average, 2);
	}
	return sum / len;
}

double Measure::CalcStd(const Eigen::MatrixXd &x, const double average)
{
	double variance = CalcVariance(x, average);
	return sqrt(variance);
}

/*!
*@brief  计算所有模型的尺寸，并以二进制保存
*@param[out]
*@param[in]  const Eigen::MatrixXd & all_vertices  所有模型的顶点信息
*@param[in]  const Eigen::Matrix3Xi & facets  三角面片信息
*@param[in]  const std::vector<std::vector<std::vector<double>>> & control_points  控制点的信息
*@param[in]  Eigen::MatrixXd & measure_lists  测量数据矩阵 shape(19, num_model)
*@return     void
*/void Measure::ConvertMeasure(const Eigen::MatrixXd & all_vertices, const Eigen::Matrix3Xi &facets,
	const std::vector<std::vector<std::vector<double>>>& control_points, Eigen::MatrixXd &measure_lists, Eigen::MatrixX3f& circum)
{
	cout << "Start convert measure..." << endl;

	clock_t t = clock();
	Measure measure;
	Eigen::MatrixXd measure_list;
	Eigen::MatrixXd verts;
	measure_lists.resize(M_NUM, all_vertices.cols());

	for (int i = 0; i < all_vertices.cols(); ++i)
	{
		verts = all_vertices.col(i);
		verts.resize(3, 12500);
		measure_list = measure.CalcMeasure(control_points, verts, facets, circum, i);
		measure_lists.col(i) = measure_list;
	}
	Eigen::MatrixXd mean_measure, std_measure;
	mean_measure.resize(M_NUM, 1);
	std_measure.resize(M_NUM, 1);

	for (int i = 0; i < M_NUM; ++i)
	{
		mean_measure(i, 0) = measure_lists.row(i).mean();
		std_measure(i, 0) = CalcStd(measure_lists.row(i), mean_measure(i, 0));
	}
	for (int i = 0; i < all_vertices.cols(); ++i)
	{
		measure_lists.col(i) = measure_lists.col(i) - mean_measure;
		measure_lists.col(i) = measure_lists.col(i).cwiseQuotient(std_measure);
	}
	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "measure_list").c_str(), measure_lists);
	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "mean_measure").c_str(), mean_measure);
	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "std_measure").c_str(), std_measure);

	cout << "Calculate measure spend: " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds." << endl;
}


/*!
*@brief  通过向量叉积判断方向
*@param[out]
*@param[in]  const pmp::vec3 & p
*@param[in]  const pmp::vec3 & q
*@param[in]  const pmp::vec3 & r
*@return     float
*/
float Measure::orientation(const pmp::vec3& p, const pmp::vec3& q, const pmp::vec3& r)
{
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
//bool Measure::compare(const pmp::vec3& p1, const pmp::vec3& p2)
//{
//	int orient = orientation(ori_point, p1, p2);
//	if (orient == 0)
//		return ((distance(ori_point, p2) >= distance(ori_point, p1)) ? 0 : 1);
//	return (orient == 1) ? 0 : 1;
//	//return (orient > 0) || (orient == 0 && (distance(ori_point, p2) >= distance(ori_point, p1)));
//}


/*!
*@brief  Toleft检测
*@param[out]
*@param[in]  vector<pmp::vec3> & v
*@param[in]  pmp::vec3 & p
*@return     std::vector<pmp::vec3>
*/
vector<pmp::vec3> Measure::keep_left(vector<pmp::vec3>& v, pmp::vec3& p)
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
*@param[in]  vector<pmp::vec3> & points  输入的点集
*@return     std::vector<pmp::vec3>  凸包点集
*/
vector<pmp::vec3> Measure::GrahamScan(vector<pmp::vec3>& points)
{
	if (points.size() <= 1)
		return points;
	//sort(points.begin(), points.end(), compare);
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



/*!
*@brief  每条边设置是否被访问标记
*@param[out]
*@param[in]  SurfaceMesh & mesh
*@return     void
*/
void Measure::SetBool(SurfaceMesh&mesh)
{
	//if (mesh.has_edge_property("e:isChecked"))
	//{
	//	auto is_checked = mesh.edge_property<bool>("e:isChecked");
	//	for (auto e : mesh.edges())
	//	{
	//		is_checked[e] = 0;
	//	}
	//}
	auto is_checked = mesh.edge_property<bool>("e:isChecked");
	for (auto e : mesh.edges())
	{
		is_checked[e] = 0;
	}
}


/*!
*@brief  计算交点坐标
*@param[out]
*@param[in]  pmp::vec3 & intersect_point
*@param[in]  const float & scale
*@param[in]  const SurfaceMesh & mesh
*@param[in]  const pmp::vec3 & p0
*@param[in]  const pmp::vec3 & p1
*@return     void
*/
void Measure::CalcIntersectionPoint(pmp::vec3& intersect_point, const float& scale, const SurfaceMesh& mesh, const pmp::vec3& p0, const pmp::vec3& p1)
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

/*!
*@brief  判断是否与直线相交
*@param[out]
*@param[in]  pmp::vec3 & intersect_point
*@param[in]  const pmp::Edge & edg
*@param[in]  const pmp::vec3 & normal
*@param[in]  const pmp::vec3 & v
*@param[in]  const SurfaceMesh & mesh
*@return     float
*/
float Measure::CheckIntersection(pmp::vec3& intersect_point, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, const SurfaceMesh & mesh)
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


/*!
*@brief  计算围长
*@param[out]
*@param[in]  std::vector<pmp::vec3> & intersect_points
*@return     float
*/
float Measure::CalcChainLength(std::vector<pmp::vec3>& intersect_points)
{
	float chain_len = 0;
	int n = intersect_points.size();
	for (int i = 1; i < n; ++i)
	{
		float len = distance(intersect_points[i - 1], intersect_points[i]);
		chain_len += len;
	}
	chain_len += distance(intersect_points[n - 1], intersect_points[0]);
	return chain_len;
}

/*!
*@brief  计算所有的交点
*@param[out]
*@param[in]  std::vector<pmp::vec3> & intersect_points
*@param[in]  std::vector<pmp::Edge> & edgs_intersect
*@param[in]  std::vector<float> & scale_set
*@param[in]  SurfaceMesh & mesh
*@param[in]  const pmp::vec3 & normal
*@param[in]  const pmp::vec3 & v
*@return     void
*/
void Measure::CalcIntersectionPoints(std::vector<pmp::vec3>& intersect_points, std::vector<pmp::Edge>& edgs_intersect, std::vector<float>& scale_set,
	SurfaceMesh& mesh, const pmp::vec3& normal, const pmp::vec3& v)
{
	pmp::vec3 intersect_point;
	//会求出三个点集，分别为中间身体的围长和手臂的围长,用一个临时数组保存其他两个点集

	float chain_len = INT_MIN, chain_len_t;

	auto is_checked = mesh.get_edge_property<bool>("e:isChecked");
	for (auto &edg : mesh.edges())
	{
		//跳过已经遍历的边
		if (is_checked[edg]) continue;

		float scale = CheckIntersection(intersect_point, edg, normal, v, mesh);
		if (scale >= 0 && scale <= 1)
		{
			std::vector<pmp::vec3> intersect_points_t;
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


/*!
*@brief  对输入的边进行邻域遍历，找到所有与平面相交的边
*@param[out]
*@param[in]  pmp::EdgeProperty<bool> & is_checked
*@param[in]  std::vector<float> & scale_set
*@param[in]  std::vector<pmp::vec3> & intersect_points
*@param[in]  std::vector<pmp::Edge> & edgs_intersect
*@param[in]  const SurfaceMesh & mesh
*@param[in]  const pmp::Edge & edg
*@param[in]  const pmp::vec3 & normal
*@param[in]  const pmp::vec3 & v
*@param[in]  pmp::vec3 intersect_point
*@return     void
*/
void Measure::FindIntersectionEdgeNearby(pmp::EdgeProperty<bool>& is_checked, std::vector<float>& scale_set, std::vector<pmp::vec3>& intersect_points,
	std::vector<pmp::Edge>& edgs_intersect, SurfaceMesh& mesh, const pmp::Edge& edg, const pmp::vec3& normal, const pmp::vec3& v, pmp::vec3 intersect_point)
{
	pmp::Halfedge half_edge = mesh.halfedge(edg, 0);
	pmp::Halfedge half_edge_next = mesh.next_halfedge(half_edge);
	while (half_edge != half_edge_next)
	{
		pmp::Edge edg_next = mesh.edge(half_edge_next);
		float scale = CheckIntersection(intersect_point, edg_next, normal, v, mesh);
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
}

float Measure::CalcConvexCircumference(SurfaceMesh& mesh, const int index)
{
	std::vector<float> scale_set;
	std::vector<pmp::Edge> edgs_interset;
	std::vector<pmp::vec3> intersect_points;
	pmp::vec3 ori_point = mesh.position(Vertex(index));
	SetBool(mesh);
	CalcIntersectionPoints(intersect_points, edgs_interset, scale_set, mesh, normal, ori_point);
	std::vector<pmp::vec3> out = GrahamScan(intersect_points);
	float circumference = CalcChainLength(out);
	return circumference;
}

std::vector<float> Measure::CalcConvexCircumferences(SurfaceMesh& mesh)
{
	std::vector<float> circumferences;
	for (auto& id : cir_index)
	{
		circumferences.push_back(CalcConvexCircumference(mesh, id));
	}
	return circumferences;
}

void Measure::CalcCircumferencesAndSave()
{
	std::string trainModelPath = DATASET_PATH;
	std::vector<std::string> trainFiles = GetFiles(trainModelPath + "*");
	Eigen::MatrixX3f circum;
	circum.resize(trainFiles.size(), 3);
	int idx = 0;
	for (auto & file_name : trainFiles)
	{
		SurfaceMesh mesh;
		mesh.read((trainModelPath + file_name).c_str());
		std::vector<float> circum_t = CalcConvexCircumferences(mesh);
		Eigen::RowVector3f row_;
		for (int i = 0; i < 3; ++i)
		{
			row_(i) = circum_t[i];
		}
		circum.block(idx++, 0, 1, 3) = row_;
	}

	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "circumferences").c_str(), circum);
}