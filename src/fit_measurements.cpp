#include "fit_measurements.h"


Eigen::Matrix3Xd verts;
Eigen::Matrix3Xi faces;
Eigen::MatrixXd input_measure;
Eigen::MatrixXd measurements;
std::vector<std::vector<int>> point_idx;
std::vector<std::vector<std::vector<double>>> control_points;


int main()
{
	Estimate estimate;
	Reshaper reshaper;
	Measure measure;
	meshio::ReadObj(ori_mesh_path, verts, faces);
	reshaper.SaveBinControlPoint(control_points);
	measurements = measure.CalcMeasure(control_points, verts, faces);
	alglib::real_1d_array xx;
	xx.attach_to_ptr(verts.cols() * 3, verts.data());
	double f(0.0);
	alglib::real_1d_array g;
	grad_function(xx, f, g, nullptr);


}

void grad_function(const alglib::real_1d_array& x, double& func, alglib::real_1d_array& grad, void* ptr)
{
	alglib::real_1d_array temp(x);
	Eigen::Map<Matrix3Xd> vertices(temp.getcontent(), 3, verts.cols());
	func = 0.0;
	Eigen::VectorXd gradient;
	gradient.setConstant(vertices.cols() * 3, 0);
	Estimate estimate;
	CalcEuclideanGradient(gradient, vertices);
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
void CalcEuclideanGradient(Eigen::VectorXd& gradient, Matrix3Xd vertices)
{
	pmp::vec3 grad;
	int id1 = point_idx[0][0], id2 = point_idx[0][1];
	pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
	pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
	grad = 4 * (distance(p1, p2)*distance(p1, p2) - input_measure.coeff(0, 0))*(p1 - p2);
	for (int i = 0; i < 3; ++i)
	{
		gradient(id1 * 3 + i) = grad[i];
		gradient(id2 * 3 + i) = -grad[i];
	}
}

/*!
*@brief
*@param[out]
*@param[in]  Eigen::MatrixXd & gradient  整体梯度
*@param[in]  SurfaceMesh & mesh  网格信息
*@param[in]  std::vector<int> point_idx  相关顶点下标
*@param[in]  Eigen::MatrixXd & input_measure  目标尺寸
*@param[in]  Eigen::MatrixXd & measure  当前网格尺寸
*@return     void
*/
void CalcGeodesicGradient(Eigen::VectorXd& gradient, Matrix3Xd vertices, Eigen::MatrixXd measure)
{
	pmp::vec3 grad;
	for (int i = 1; i < point_idx.size(); ++i)
	{
		for (int j = 1; j < point_idx[i].size(); ++j)
		{
			int id1 = point_idx[i][i - 1], id2 = point_idx[i][i];
			pmp::vec3 p1; p1[0] = vertices.coeff(0, id1); p1[1] = vertices.coeff(1, id1); p1[2] = vertices.coeff(2, id1);
			pmp::vec3 p2; p2[0] = vertices.coeff(0, id2); p2[1] = vertices.coeff(1, id2); p2[2] = vertices.coeff(2, id2);
			float cur_len = distance(p1, p2);
			grad = 4 * (std::pow(cur_len, 2) - CalcTargetLen(measure, cur_len, i))*(p1 - p2);
			for (int i = 0; i < 3; ++i)
			{
				gradient(id1 * 3 + i) += grad[i];
				gradient(id2 * 3 + i) += -grad[i];
			}
		}
	}
}

/*!
*@brief  求目标边长
*@param[out]
*@param[in]  Eigen::MatrixXd & input_measure  目标总长
*@param[in]  Eigen::MatrixXd & measure  已知总长
*@param[in]  const float cur_len  已知边长
*@param[in]  int index  第index个尺寸
*@return     float
*/
float CalcTargetLen(Eigen::MatrixXd& measure, const float cur_len, int index)
{
	float target_len = (cur_len / measure.coeff(index, 0))*(input_measure.coeff(index, 0));
	return target_len;
}