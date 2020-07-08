#pragma once
#include <vector>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Python.h>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <unsupported/Eigen/SparseExtra>
#include <unsupported/Eigen/KroneckerProduct>

#include "utils.h"
#include "mesh_io.h"
#include "binary_io.h"

namespace py = pybind11;


class Reshaper
{
public:
	Reshaper();
	~Reshaper();
	void SaveBinControlPoint(std::vector<std::vector<std::vector<double>>> &control_points);
	void SaveBinEdge(std::vector<std::vector<std::vector<double>>>& control_points, std::vector<std::vector<int>> &point_idx);
	void FitOneMeasurements(Eigen::Matrix3Xd & res, std::vector<int> point_idx, const Eigen::Matrix3Xd & vertices, const double measurement);
	void FitMeasurements(Eigen::Matrix3Xd& res_verts, std::vector<std::vector<int>> point_idx, const Eigen::Matrix3Xd &vertices, const Eigen::MatrixXd measurements);
	void SaveBinVerts(const char *filename, const std::string &path, const std::vector<std::string> &files);
	void SaveBinFaces(const char *filename, const std::string &path, const std::vector<std::string> &files);
	void SaveVertFacetInBin();
	void Reshaper::CalcMeanMesh(const char * filename, const Eigen::MatrixXd &vertices, const Eigen::Matrix3Xi &facets, Eigen::Matrix3Xd &undeform_mesh);
	void Reshaper::GetKnownUndeformedVerticeInverse(Eigen::MatrixX3d &undeform_mesh, const Eigen::Matrix3Xi &facets, Eigen::MatrixXd &v_inverse);
	void Reshaper::SaveDataInBinary(Eigen::MatrixXd &vertices, const Eigen::Matrix3Xi &facets, Eigen::MatrixXd v_inverse,
		Eigen::MatrixXd &Q_transformation, Eigen::MatrixXd &Q_determinant, Eigen::MatrixXd &mean_deform, Eigen::MatrixXd &std_deform);
	double CalcStd(const Eigen::MatrixXd &x, const double average);
	double CalcVariance(const Eigen::MatrixXd &x, const double average);
	Eigen::MatrixXd Reshaper::GetOneModelTransformation(Eigen::MatrixXd &deformed_mesh, const Eigen::Matrix3Xi &facets);
	void GetTransformationBasis(Eigen::MatrixXd &transformation, Eigen::MatrixXd &coefficient, Eigen::MatrixXd &basis);
	void Reshaper::GetMeasure2Deform(Eigen::MatrixXd &coefficient, Eigen::MatrixXd &measurelist, Eigen::MatrixXd &measure2deform);
	void ConstructMatrix(const Eigen::MatrixXd &undeform_mesh_, const Eigen::Matrix3Xi &facets, Eigen::SparseMatrix<double> &A);
	void QRFactorize(const Eigen::MatrixXd &a, Eigen::MatrixXd &q, Eigen::MatrixXd &r);

	void Reshaper::Synthesize(Eigen::SparseMatrix<double> A, Eigen::MatrixXd deform, Eigen::Matrix3Xi &facets);
	void Reshaper::RFEMapping(Eigen::MatrixXd input_measure, Eigen::SparseMatrix<double> A, Eigen::Matrix3Xi &facets);
private:

};
