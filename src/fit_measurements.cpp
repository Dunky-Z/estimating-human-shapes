#include<iomanip>

#include "fit_measurements.h"

using namespace Eigen;
using namespace pmp;
using namespace std;


const int M_NUM = 15;
const double step = 1;
Eigen::Matrix3Xd verts;
Eigen::Matrix3Xi faces;
Eigen::MatrixXd input_m(M_NUM, 1);
Eigen::VectorXd gradient;

Eigen::MatrixXd measurements;
std::vector<std::vector<int>> point_idx;
std::vector<std::vector<std::vector<double>>> control_points;


int main()
{
	Reshaper reshaper;
	Measure measure;
	meshio::ReadObj(ori_mesh_path, verts, faces);
	verts = verts * 100;
	std::cout << verts.middleCols<30>(280) << std::endl;
	std::cout << "----------------------------" << std::endl;
	reshaper.SaveBinControlPoint(control_points);
	reshaper.SaveBinEdge(control_points, point_idx);
	input_m << 1795.61, 460.47, 890.41, 823.41, 419.05, 824.58, 1126.35, 1199.55, 1336.46, 649.92, 623.889, 204.25, 1313.27, 442.89, 726.47;
	input_m = input_m * 100;
	measurements = measure.CalcMeasure(control_points, verts, faces);

	//alglib::real_1d_array xx;
	//xx.attach_to_ptr(verts.cols() * 3, verts.data());
	//double f(0.0);
	//alglib::real_1d_array g;
	//grad_function(xx, f, g, nullptr);

	alglib::real_1d_array x;
	x.attach_to_ptr(verts.cols() * 3, verts.data());
	double epsg = 0.0;
	double epsf = 0.0;
	double epsx = 0.0;
	double stpmax = 0.0;
	alglib::ae_int_t maxits = 0;
	alglib::minlbfgsstate state;
	alglib::minlbfgsreport rep;

	alglib::minlbfgscreate(5.0, x, state);
	alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
	alglib::minlbfgssetstpmax(state, stpmax);
	Eigen::VectorXd vs;
	vs.resize(verts.cols() * 3);
	memset(vs.data(), 100, sizeof(double) * vs.size());
	alglib::real_1d_array scalar;
	scalar.setcontent(vs.size(), vs.data());
	alglib::minlbfgssetscale(state, scalar);

	////OptGuard is essential at the early prototyping stages.
	// first run
	alglib::minlbfgsoptimize(state, grad_function);
	alglib::real_1d_array rex;
	alglib::minlbfgsresults(state, rex, rep);


	std::cout << "迭代次数 : " << rep.iterationscount << std::endl;
	std::cout << "梯度计算次数 : " << rep.nfev << std::endl;
	std::cout << "终止情况 : " << rep.terminationtype << std::endl;

	Eigen::Matrix3Xd curV = Eigen::Map<Eigen::Matrix3Xd>(rex.getcontent(), 3, verts.cols());
	cout << curV.middleCols<30>(280) << endl;

	//将梯度转成矩阵形式
	Eigen::Matrix3Xd gradient_t;
	gradient_t.resize(3, verts.cols());
	for (size_t i = 0; i < verts.cols(); ++i)
	{
		for (size_t j = 0; j < 3; ++j)
		{
			gradient_t(j, i) = gradient(3 * i + j);
		}
	}

	//std::cout << gradient_t.middleCols<10>(280) << std::endl;
	//计算能量
	//double energy = 0.0;

	//CalcEnergy(energy, verts);

	//std::cout << "-------------------" << endl;
	////std::cout << energy << std::endl;

	//Eigen::Matrix3Xd t_grad;
	//t_grad.setConstant(3, verts.cols(), 0);
	//for (size_t i = 0; i < verts.cols(); ++i)
	//{
	//	for (size_t j = 0; j < 3; ++j)
	//	{
	//		double tmp = 0.0;
	//		verts(j, i) += step;
	//		CalcEnergy(tmp, verts);
	//		//cout << tmp << endl;
	//		t_grad(j, i) = (tmp - energy) / step;
	//		//if (i > 284)
	//		//{
	//		//	std::cout << tmp << "-" << energy << "/" << step << std::endl;
	//		//	std::cout << setprecision(7);
	//		//	std::cout << t_grad.middleCols<10>(280) << std::endl;
	//		//	std::cout << "----" << std::endl;
	//		//}
	//		verts(j, i) -= step;

	//	}
	//}
	//std::cout << t_grad.leftCols(300) << std::endl;
	//std::cout << t_grad - gradient_t << std::endl;
}

void CalcEnergy(double& enyg, Eigen::Matrix3Xd& vertices)
{
	for (size_t i_ = 1, i = 1; i_ < point_idx.size(); ++i_)
	{
		//剔除三个围长
		if (i_ == 2 || i_ == 3 || i_ == 4)
		{
			continue;
		}
		size_t n = point_idx[i_].size();
		for (size_t j = 0; j < n - 1; ++j)
		{
			int id1 = point_idx[i_][j], id2 = point_idx[i_][j + 1];
			pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
			pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
			double cur_len = distance(p1, p2);
			double p = std::pow(cur_len, 2);
			double l = std::pow(CalcTargetLen(measurements, cur_len, i), 2);
			enyg = enyg + std::pow(p - l, 2);
			std::cout << setprecision(15);
			//std::cout << std::fixed << p << "  " << l << "  " << enyg << std::endl;
		}
		i++;
	}
}


void grad_function(const alglib::real_1d_array& x, double& func, alglib::real_1d_array& grad, void* ptr)
{
	alglib::real_1d_array temp(x);
	Eigen::Map<Matrix3Xd> vertices(temp.getcontent(), 3, verts.cols());
	func = 0.0;
	gradient.setConstant(vertices.cols() * 3, 0);
	//CalcEuclideanGradient(gradient, vertices);
	CalcGeodesicGradient(gradient, vertices, measurements);
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
//void CalcEuclideanGradient(Eigen::VectorXd& gradient, Eigen::Matrix3Xd& vertices)
//{
//	pmp::vec3 grad;
//	int id1 = point_idx[0][0], id2 = point_idx[0][1];
//	pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
//	pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
//	grad = 4 * (std::pow(distance(p1, p2), 2) - std::pow(input_m.coeff(0, 0), 2))*(p1 - p2);
//	for (int i = 0; i < 3; ++i)
//	{
//		gradient(id1 * 3 + i) = grad[i];
//		gradient(id2 * 3 + i) = -grad[i];
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
void CalcGeodesicGradient(Eigen::VectorXd& gradient, Matrix3Xd vertices, Eigen::MatrixXd& measurements)
{
	pmp::vec3 grad;
	//从1开始因为poin_idx[0]是欧式距离
	for (size_t i_ = 1, i = 1; i_ < point_idx.size(); ++i_)
	{
		if (i_ == 2 || i_ == 3 || i_ == 4)
		{
			continue;
		}
		size_t n = point_idx[i_].size();
		for (size_t j = 0; j < n - 1; ++j)
		{
			int id1 = point_idx[i_][j], id2 = point_idx[i_][j + 1];
			//std::cout << id1 << "  " << id2 << std::endl;
			pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
			pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
			float cur_len = distance(p1, p2);
			grad = 4 * (std::pow(cur_len, 2) - std::pow(CalcTargetLen(measurements, cur_len, i), 2))*(p1 - p2);
			for (size_t k = 0; k < 3; ++k)
			{
				gradient(id1 * 3 + k) += grad[k];
				gradient(id2 * 3 + k) += -grad[k];
			}
		}
		++i;
	}
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