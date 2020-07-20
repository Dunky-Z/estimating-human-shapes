#pragma once
#include <Eigen/Dense>

#include "measure.h"
#include "reshaper.h"
#include "utils.h"

using namespace Eigen;
using namespace pmp;
using namespace std;

void grad_function(const alglib::real_1d_array& x, double& func, alglib::real_1d_array& grad, void* ptr);
void CalcEuclideanGradient(Eigen::VectorXd& gradient, Matrix3Xd vertices);
void CalcGeodesicGradient(Eigen::VectorXd& gradient, Matrix3Xd vertices, Eigen::MatrixXd measurements);
float CalcTargetLen(Eigen::MatrixXd measurements, const float cur_len, const int index);
void CalcEnergy(double& energy, Eigen::Matrix3Xd vertices);