#include<iomanip>
#include <iostream>
#include <io.h>

#include "fit_measurements.h"


using namespace Eigen;
using namespace pmp;
using namespace std;


const int M_NUM = 18;
const double step = 1;
Eigen::Matrix3Xd verts;
Eigen::Matrix3Xi faces;
Eigen::MatrixXd input_m(M_NUM, 1);
Eigen::VectorXd gradient;

Eigen::MatrixXd measurements;
std::vector<std::vector<int>> point_idx;
std::vector<std::vector<std::vector<double>>> control_points;

vector<double> length;
vector<double> circle;

int count_iter = 0;

int main()
{
	//Reshaper reshaper;
	//reshaper.SaveBinControlPoint(control_points);
	//reshaper.SaveBinEdge(control_points, point_idx);
	Measurement measure;
	meshio::ReadObj(ori_mesh_path, verts, faces);
	//measure.CalcGeodesicAndCircum(verts, faces, 0);
	ReadPathLength(length, circle);
	input_m << 1701.61, 371.47, 845.81, 762.78, 784.35, 744.41, 790.41, 301.05, 740.58, 755.354, 955.55, 1185.46, 541.92, 265.889, 166.25, 1260.27, 374.89, 518.47;
	int num_verts = verts.cols();
	alglib::real_1d_array x;
	//std::cout << vertices.coeff(0, 0) << "  " << vertices.coeff(1, 0) << "  " << vertices.coeff(2, 0) << std::endl;
	x.attach_to_ptr(3 * num_verts, verts.data());
	Eigen::VectorXd scale;
	scale.resize(3 * num_verts); scale.setOnes();
	alglib::real_1d_array s;
	s.setcontent(3 * num_verts, scale.data());
	double epsg = 1e-5;
	double epsf = 1e-5;
	double epsx = 1e-5;

	alglib::ae_int_t maxits = 5;
	alglib::minlbfgsstate state;


	alglib::minlbfgscreate(1, x, state);
	alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
	alglib::minlbfgssetscale(state, s);

	minlbfgsoptguardsmoothness(state);
	minlbfgsoptguardgradient(state, 0.001);
	alglib::minlbfgsreport rep;

	alglib::minlbfgsoptimize(state, grad_function);
	alglib::minlbfgsresults(state, x, rep);


	std::cout << "迭代次数 : " << rep.iterationscount << std::endl;
	std::cout << "梯度计算次数 : " << rep.nfev << std::endl;
	std::cout << "终止情况 : " << rep.terminationtype << std::endl;

	Eigen::Matrix3Xd curV = Eigen::Map<Eigen::Matrix3Xd>(x.getcontent(), 3, verts.cols());
	meshio::SaveObj("../data/res.obj", curV, faces);

}

void CalcAndSaveMeasure()
{
	Measurement measure;
	Eigen::Matrix3Xd V;
	Eigen::Matrix3Xi F;
	meshio::ReadObj((BIN_DATA_PATH + "AVE.obj").c_str(), V, F);
	measure.CalcGeodesicAndCircum(V, F, 1);
	std::cout << "CalcAndSaveMeasure Done" << std::endl;
}


void ReadPathLength(
	std::vector<double>& length,
	std::vector<double>& circle)
{
	ifstream in_1("../data/path/length.txt");
	for (int i = 0; i < 5; ++i)
	{
		double dist;
		in_1 >> dist;
		length.push_back(dist);
	}
	in_1.close();
	ifstream in_2("../data/path/circle.txt");
	for (int i = 0; i < 12; ++i)
	{
		double dist;
		in_2 >> dist;
		circle.push_back(dist);
	}
	in_2.close();
}

void grad_function(const alglib::real_1d_array& x, double& func, alglib::real_1d_array& grad, void* ptr)
{
	std::cout << "Num of iterations: " << ++count_iter << std::endl;

	int num_verts = verts.cols();
	alglib::real_1d_array temp(x);
	Eigen::Matrix3Xd vertices = Eigen::Map<Eigen::Matrix3Xd>(temp.getcontent(), 3, num_verts);
	func = 0.0;
	gradient.setConstant(vertices.cols() * 3, 0);
	CalcEuclideanGradient(func, gradient, vertices);
	CalcGeodesicGradient(func, gradient, vertices);
	CalcCircumferenceGradient(func, gradient, vertices);
	for (int i = 0; i < grad.length(); ++i)
	{
		grad[i] = gradient(i);
	}
}

/*!
*@brief  计算欧式距离的梯度
*@param[out]
*@param[in]  Eigen::MatrixXd & gradient
*@param[in]  SurfaceMesh & mesh
*@param[in]  std::vector<int> point_idx
*@param[in]  Eigen::MatrixXd & input_measure
*@return     void
*/
void CalcEuclideanGradient(
	double& energy,
	Eigen::VectorXd& gradient,
	Eigen::Matrix3Xd& vertices)
{
	pmp::vec3 grad;
	int id1 = 12480, id2 = 58;//12480头顶58脚底
	pmp::vec3 p1 = vertices.col(id1);
	pmp::vec3 p2 = vertices.col(id2);
	auto input = std::pow(input_m.coeff(0, 0) / 1000, 2);
	grad = 4 * (std::pow(distance(p1, p2), 2) - input)*(p1 - p2);
	energy = std::pow((std::pow(distance(p1, p2), 2) - input), 2);
	for (int i = 0; i < 3; ++i)
	{
		gradient(id1 * 3 + i) += grad[i];
		gradient(id2 * 3 + i) += -grad[i];
	}
	std::cout << ">>>> CalcEuclideanGradient Done!" << std::endl;
}


/*!
*@brief  计算围长距离的梯度
*@param[out]
*@param[in]  Eigen::VectorXd & gradient
*@param[in]  Matrix3Xd vertices
*@param[in]  Eigen::MatrixXd measurements
*@return     void
*/
void CalcGeodesicGradient(
	double& energy,
	Eigen::VectorXd& gradient,
	Matrix3Xd& vertices)
{
	pmp::vec3 grad;
	Measurement measure;
	std::vector<node> path;
	for (int i = 0; i < measure.GetNumOfGeodetic(); ++i)
	{

		ifstream in("../data/path/" + measure.SemanticLable[i] + to_string(0), ios::binary);
		int len;
		in.read((char*)(&len), sizeof(int));
		for (int i = 0; i < len; ++i)
		{
			int x, y;
			double t;
			in.read((char*)(&x), sizeof(int));
			in.read((char*)(&y), sizeof(int));
			in.read((char*)(&t), sizeof(double));
			path.push_back({ x, y, t });
			//cout << x << " " << y << " " << t << endl;
		}
		in.close();
		for (int k = 0; k < path.size(); ++k)
		{
			int id1 = path[k].x, id2 = path[(k + 1) % path.size()].x;
			//std::cout << id1 << "  " << id2 << std::endl;
			pmp::vec3 p1 = vertices.col(id1);
			pmp::vec3 p2 = vertices.col(id2);
			double dist = distance(p1, p2);
			double cur_len = std::pow(dist, 2);
			double tar_len = std::pow(CalcTargetLen(length, dist, i), 2);
			energy += std::pow(cur_len - tar_len, 2);
			grad = 4 * (cur_len - tar_len)*(p2 - p1);
			for (size_t l = 0; l < 3; ++l)
			{
				gradient(id1 * 3 + l) += grad[l];
				gradient(id2 * 3 + l) += -grad[l];
			}
		}
	}
	std::cout << ">>>> CalcGeodesicGradient Done!" << std::endl;
}

/*!
*@brief  计算测地距离的梯度
*@param[out]
*@param[in]  Eigen::VectorXd & gradient
*@param[in]  Matrix3Xd vertices
*@param[in]  Eigen::MatrixXd measurements
*@return     void
*/
void CalcCircumferenceGradient(
	double& energy,
	Eigen::VectorXd& gradient,
	Matrix3Xd& vertices)
{
	pmp::vec3 grad;
	Measurement measure;
	std::vector<node> path;
	int M = measure.GetNumOfGeodetic();
	for (int i = 0; i < measure.GetNumOfCircumference(); ++i)
	{
		for (size_t j = 0; j < 4; ++j)
		{
			ifstream in("../data/path/" + measure.SemanticLable[M+i] + to_string(j), ios::binary);
			int len;
			in.read((char*)(&len), sizeof(int));
			for (int i = 0; i < len; ++i)
			{
				int x, y;
				double t;
				in.read((char*)(&x), sizeof(int));
				in.read((char*)(&y), sizeof(int));
				in.read((char*)(&t), sizeof(double));
				path.push_back({ x, y, t });
			}
			in.close();
			for (int k = 0; k < path.size(); ++k)
			{
				int id1 = path[k].x, id2 = path[k].y;
				int id3 = path[(k + 1) % path.size()].x, id4 = path[(k + 1) % path.size()].y;
				double t1 = path[k].t, t2 = path[(k + 1)%path.size()].t;
				//std::cout << id1 << "  " << id2 << std::endl;
				pmp::vec3 p1 = vertices.col(id1);
				pmp::vec3 p2 = vertices.col(id2);
				pmp::vec3 p_mid_0 = t1 * (p2 - p1) + p1;
				pmp::vec3 p3 = vertices.col(id3);
				pmp::vec3 p4 = vertices.col(id4);
				pmp::vec3 p_mid_1 = t2 * (p4 - p3) + p3;
				double dist = distance(p_mid_0, p_mid_1);
				double cur_len = std::pow(dist, 2);
				double tar_len = std::pow(CalcTargetLen(circle, dist, i), 2);
				energy += std::pow(cur_len - tar_len, 2);
				grad = 4 * (cur_len - tar_len)*(p_mid_0 - p_mid_1);
				for (size_t l = 0; l < 3; ++l)
				{
					gradient(id1 * 3 + l) += t1 * grad[l];
					gradient(id2 * 3 + l) += -t2 * grad[l];
					gradient(id3 * 3 + l) += -t2 * grad[l];
					gradient(id4 * 3 + l) += t1 * grad[l];
				}
			}
		}
	}
	std::cout << ">>>> CalcCircumferenceGradient Done!" << std::endl;
}

/*!
*@brief  计算目标边长
*@param[out]
*@param[in]  Eigen::MatrixXd measurements
*@param[in]  const float cur_len  当前网格上这条边长
*@param[in]  int index  第index个尺寸
*@return     float
*/
float CalcTargetLen(Eigen::MatrixXd& measurements, const float& cur_len, const int& index)
{
	float target_len = (cur_len / measurements.coeff(index, 0))*(input_m.coeff(index, 0));
	return target_len;
}


/*!
*@brief  计算目标边长
*@param[out]
*@param[in]  Eigen::MatrixXd measurements
*@param[in]  const float cur_len  当前网格上这条边长
*@param[in]  int index  第index个尺寸
*@return     float
*/
double CalcTargetLen(
	const std::vector<double>& measurments,
	const double& cur_len,
	const int& index)
{
	double m = measurments[index];
	double cur_len_p = cur_len / m;
	double target_len_p = input_m.coeff(index, 0) / 1000.0;
	double target_len = cur_len_p * target_len_p;
	return target_len;
}

/*!
*@brief  计算拉普拉斯权值矩阵
*@param[out] 拉普拉斯稀疏矩阵
*@param[in]  const SurfaceMesh & mesh  待求拉普拉斯矩阵的原始网格
*@param[in]  Eigen::SparseMatrix<float> & L  拉普拉斯矩阵，是个大型稀疏矩阵
*@return     void
*/
void CaculateLaplacianCotMatrix(const SurfaceMesh& mesh, Eigen::SparseMatrix<double> & L)
{
	std::vector<Tri> tripletlist;
	tripletlist.reserve(20);
	const int p_num = mesh.n_vertices();
	L.resize(p_num, p_num);
	for (auto fit : mesh.faces())
	{
		vec3 p[3];
		float cot[3];
		int id[3];
		auto vf = mesh.vertices(fit);
		for (int i = 0; i < 3; ++i, ++vf)
		{
			p[i] = mesh.position(*vf);
			id[i] = (*vf).idx();
		}
		for (int i = 0; i < 3; ++i)
		{
			int j = (i + 1) % 3, k = (j + 1) % 3;
			cot[i] = dot(p[j] - p[i], p[k] - p[i]) / norm(cross(p[j] - p[i], p[k] - p[i]));

			tripletlist.push_back(Tri(id[j], id[k], -0.5 * cot[i]));
			tripletlist.push_back(Tri(id[k], id[j], -0.5 * cot[i]));
		}
		for (int i = 0; i < 3; ++i)
		{
			tripletlist.push_back(Tri(id[i], id[i], 0.5*(cot[(i + 1) % 3] + cot[(i + 2) % 3])));
		}
	}
	L.setFromTriplets(tripletlist.begin(), tripletlist.end());
}

void CaculateCoefficientCotMatrix(const SurfaceMesh& mesh, Eigen::SparseMatrix<double> & A)
{

}

//void CalcEnergy(double& enyg, Eigen::Matrix3Xd& vertices)
//{
//	for (size_t i_ = 1, i = 1; i_ < point_idx.size(); ++i_)
//	{
//		//剔除三个围长
//		if (i_ == 2 || i_ == 3 || i_ == 4)
//		{
//			continue;
//		}
//		size_t n = point_idx[i_].size();
//		for (size_t j = 0; j < n - 1; ++j)
//		{
//			int id1 = point_idx[i_][j], id2 = point_idx[i_][j + 1];
//			pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
//			pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
//			double cur_len = distance(p1, p2);
//			double p = std::pow(cur_len, 2);
//			double l = std::pow(CalcTargetLen(measurements, cur_len, i), 2);
//			enyg = enyg + std::pow(p - l, 2);
//			std::cout << setprecision(15);
//			//std::cout << std::fixed << p << "  " << l << "  " << enyg << std::endl;
//		}
//		i++;
//	}
//}

/*!
*@brief  计算测地距离的梯度
*@param[out]
*@param[in]  Eigen::VectorXd & gradient
*@param[in]  Matrix3Xd vertices
*@param[in]  Eigen::MatrixXd measurements
*@return     void
*/
//void CalcGeodesicGradient(Eigen::VectorXd& gradient, Matrix3Xd vertices, Eigen::MatrixXd& measurements)
//{
//	pmp::vec3 grad;
//	//从1开始因为poin_idx[0]是欧式距离
//	for (size_t i_ = 1, i = 1; i_ < point_idx.size(); ++i_)
//	{
//		if (i_ == 2 || i_ == 3 || i_ == 4)
//		{
//			continue;
//		}
//		size_t n = point_idx[i_].size();
//		for (size_t j = 0; j < n - 1; ++j)
//		{
//			int id1 = point_idx[i_][j], id2 = point_idx[i_][j + 1];
//			//std::cout << id1 << "  " << id2 << std::endl;
//			pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
//			pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
//			float cur_len = distance(p1, p2);
//			grad = 4 * (std::pow(cur_len, 2) - std::pow(CalcTargetLen(measurements, cur_len, i), 2))*(p1 - p2);
//			for (size_t k = 0; k < 3; ++k)
//			{
//				gradient(id1 * 3 + k) += grad[k];
//				gradient(id2 * 3 + k) += -grad[k];
//			}
//		}
//		++i;
//	}
//}
