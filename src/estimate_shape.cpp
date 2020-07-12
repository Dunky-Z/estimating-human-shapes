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
	//std::vector<float> scale_set;
	//std::vector<pmp::Edge> edgs_intersect;
	//std::vector<pmp::vec3> intersect_points;
	//cout << mesh.n_vertices() << endl;

	//estimate.SetBool(mesh);
	//estimate.CalcIntersectionPoints(intersect_points, edgs_intersect, scale_set, mesh, normal, ori_point);

	////SavePointToFile(filepath, intersect_points);
	//vector<pmp::vec3> output = GrahamScan(intersect_points);
	//SavePointToFile(convex_hull_path, output);
	/*--------------------------------------------------------------------------*/
	Measure mesaure;

	mesaure.CalcCircumferencesAndSave();
}

/*!
*@brief  ���������ݶ�
*@param[out] 
*@param[in]  Eigen::MatrixXd & gradient  [N,3]
*@param[in]  SurfaceMesh & mesh  
*@param[in]  Eigen::MatrixXd & input_measure  
*@param[in]  std::vector<std::vector<int>> point_idx  
*@param[in]  Eigen::MatrixXd & measure  ��ǰ����ߴ���Ϣ
*@return     void  
*/
void Estimate::CalcGradient(std::vector<std::vector<int>> point_idx, Eigen::MatrixXd& measure)
{
	Eigen::MatrixXd gradient;

	CalcEuclideanGradient(gradient, mesh, point_idx[0], input_measure);
	CalcGeodesicGradient(gradient, mesh, point_idx, input_measure, measure);
} 


/*!
*@brief  ����ŷʽ������ݶ�
*@param[out] 
*@param[in]  Eigen::MatrixXd & gradient  
*@param[in]  SurfaceMesh & mesh  
*@param[in]  std::vector<int> point_idx  
*@param[in]  Eigen::MatrixXd & input_measure  
*@return     void  
*/
void Estimate::CalcEuclideanGradient(Eigen::MatrixXd& gradient, SurfaceMesh& mesh, std::vector<int> point_idx, Eigen::MatrixXd& input_measure)
{
	pmp::vec3 grad;
	int id1 = point_idx[0], id2 = point_idx[1];
	pmp::vec3 p1 = mesh.position(Vertex(id1));
	pmp::vec3 p2 = mesh.position(Vertex(id2));
	grad = 4 * (distance(p1, p2)*distance(p1, p2) - input_measure.coeff(0, 0))*(p1 - p2);
	for (int i = 0; i < 3; ++i)
	{
		gradient.coeffRef(id1, i) = grad[i];
		gradient.coeffRef(id2, i) = -grad[i];
	}
}

/*!
*@brief
*@param[out]
*@param[in]  Eigen::MatrixXd & gradient  �����ݶ�
*@param[in]  SurfaceMesh & mesh  ������Ϣ
*@param[in]  std::vector<int> point_idx  ��ض����±�
*@param[in]  Eigen::MatrixXd & input_measure  Ŀ��ߴ�
*@param[in]  Eigen::MatrixXd & measure  ��ǰ����ߴ�
*@return     void
*/
void Estimate::CalcGeodesicGradient(Eigen::MatrixXd& gradient, SurfaceMesh& mesh, std::vector<std::vector<int>> point_idx, Eigen::MatrixXd& input_measure, Eigen::MatrixXd& measure)
{
	pmp::vec3 grad;
	for (int i = 1; i < point_idx.size(); ++i)
	{
		for (int j = 1; j < point_idx[i].size(); ++j)
		{
			int id1 = point_idx[i][i - 1], id2 = point_idx[i][i];
			pmp::vec3 p1 = mesh.position(Vertex(id1));
			pmp::vec3 p2 = mesh.position(Vertex(id2));
			float cur_len = distance(p1, p2);
			grad = 4 * (std::pow(cur_len, 2) - CalcTargetLen(input_measure, measure, cur_len, i))*(p1 - p2);
			for (int i = 0; i < 3; ++i)
			{
				gradient.coeffRef(id1, i) += grad[i];
				gradient.coeffRef(id2, i) += -grad[i];
			}
		}
	}
}

/*!
*@brief  ��Ŀ��߳�
*@param[out]
*@param[in]  Eigen::MatrixXd & input_measure  Ŀ���ܳ�
*@param[in]  Eigen::MatrixXd & measure  ��֪�ܳ�
*@param[in]  const float cur_len  ��֪�߳�
*@param[in]  int index  ��index���ߴ�
*@return     float
*/
float Estimate::CalcTargetLen(Eigen::MatrixXd& input_measure, Eigen::MatrixXd& measure, const float cur_len, int index)
{
	float target_len = (cur_len / measure.coeff(index, 0))*(input_measure.coeff(index, 0));
	return target_len;
}