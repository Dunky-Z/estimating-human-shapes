#pragma once

#include <Eigen/Dense>
#include "pmp/SurfaceMesh.h"
#include "../alglib/cpp/src/optimization.h"

#include "measure.h"

using namespace Eigen;
using namespace pmp;
using namespace std;

class Estimate
{
public:
	Estimate();
	~Estimate();
	void Apply();
	float Estimate::CalcTargetLen(Eigen::MatrixXd& input_measure, Eigen::MatrixXd& measure, const float cur_len, int index);
	void Estimate::CalcGeodesicGradient(Eigen::MatrixXd& gradient, SurfaceMesh& mesh, std::vector<std::vector<int>> point_idx, Eigen::MatrixXd& input_measure, Eigen::MatrixXd& measure);
	void Estimate::CalcEuclideanGradient(Eigen::MatrixXd& gradient, SurfaceMesh& mesh, std::vector<int> point_idx, Eigen::MatrixXd& input_measure);
	void Estimate::CalcGradient(Eigen::MatrixXd& gradient, SurfaceMesh& mesh, Eigen::MatrixXd& input_measure, std::vector<std::vector<int>> point_idx, Eigen::MatrixXd& measure);
};
                                                