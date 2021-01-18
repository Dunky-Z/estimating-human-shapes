#pragma once
#include <Eigen/Dense>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mesh_io.h"
#include "measure.h"
#include "measure_t.h"
#include "reshaper.h"
#include "utils.h"
#include "../alglib/cpp/src/optimization.h"

using namespace Eigen;
using namespace pmp;
using namespace std;

typedef Eigen::Triplet<float> Tri;

void grad_function(const alglib::real_1d_array& x, double& func, alglib::real_1d_array& grad, void* ptr);
void CalcEuclideanGradient(
	double& energy,
	Eigen::VectorXd& gradient,
	Matrix3Xd& vertices);
void CalcGeodesicGradient(
	double& energy,
	Eigen::VectorXd& gradient,
	Matrix3Xd& vertices);
void CalcCircumferenceGradient(
	double& energy,
	Eigen::VectorXd& gradient,
	Matrix3Xd& vertices);
float CalcTargetLen(Eigen::MatrixXd& measurements, const float& cur_len, const int& index);
double CalcTargetLen(
	const std::vector<double>& measurments,
	const double& cur_len,
	const int& index1,
	const int& index2);
void CalcEnergy(double& energy, Eigen::Matrix3Xd& vertices);
void CaculateLaplacianCotMatrix(const SurfaceMesh& mesh, Eigen::SparseMatrix<double> & L);
void CalcAndSaveMeasure();
void CalcSmoothnessEnergyAndGradient(
	double& energy_s,
	Eigen::VectorXd& gradient,
	Matrix3Xd& new_vertices);
void ReadPathLength(
	std::vector<double>& length,
	std::vector<double>& circle);
