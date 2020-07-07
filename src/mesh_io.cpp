#include <vector>
#include <fstream>
#include <iostream>

#include "mesh_io.h"
using namespace std;

namespace meshio {
	int ReadObj(const std::string &filename, Eigen::Matrix3Xd &V, Eigen::Matrix3Xi &F)
	{
		std::ifstream is(filename.c_str());
		if (!is)
			return 0;
		std::vector<double>  vs;
		std::vector<int>  fs;
		std::string line, pair[3];
		double  node[3];
		int  tri;
		while (!is.eof())
		{
			std::getline(is, line);
			if (line.empty() || 13 == line[0])
				continue;
			std::istringstream instream(line);
			std::string word;
			instream >> word;
			if ("v" == word || "V" == word) {
				instream >> node[0] >> node[1] >> node[2];
				for (size_t j = 0; j < 3; ++j) {
					vs.push_back(node[j]);
				}
			}
			else if ('f' == word[0] || 'F' == word[0])
			{
				instream >> pair[0] >> pair[1] >> pair[2];
				for (size_t j = 0; j < 3; ++j) {
					tri = strtoul(pair[j].c_str(), NULL, 10) - 1;
					fs.push_back(tri);
				}
			}
		}
		is.close();
		V.resize(3, vs.size() / 3);
		for (size_t i = 0, k = 0; i < V.cols(); ++i) {
			for (size_t j = 0; j < 3; ++j) {
				V(j, i) = vs[k++];
			}
		}
		F.resize(3, fs.size() / 3);
		for (size_t i = 0, k = 0; i < F.cols(); ++i) {
			for (size_t j = 0; j < 3; ++j) {
				F(j, i) = fs[k++];
			}
		}
		return 1;
	}

	int ReadObj(const std::string & filename, Eigen::MatrixXd & V)
	{
		std::ifstream is(filename.c_str());
		if (!is)
			return 0;
		std::vector<double>  vs;
		std::string line;
		double  node[6];
		while (!is.eof())
		{
			std::getline(is, line);
			if (line.empty() || 13 == line[0])
				continue;
			std::istringstream instream(line);
			std::string word;
			instream >> word;
			if ("v" == word || "V" == word)
			{
				instream >> node[0] >> node[1] >> node[2] >> node[3] >> node[4] >> node[5];
				for (size_t j = 0; j < 6; ++j)
				{
					vs.push_back(node[j]);
				}
			}
		}
		is.close();
		cout << vs.size() << endl;
		V.resize(6, vs.size() / 6);

		for (size_t i = 0, k = 0; i < V.cols(); ++i)
		{
			for (size_t j = 0; j < 6; ++j) {
				V(j, i) = vs[k++];
			}
		}
		return 1;
	}


	int SaveObj(const std::string &filename, const Eigen::Matrix3Xd &nods, const Eigen::Matrix3Xi &tris)
	{
		std::ofstream os(filename.c_str());
		if (!os)
			return 0;
		for (size_t i = 0; i < nods.cols(); ++i) {
			os << "v " << nods.col(i).transpose() << "\n";
		}

		for (size_t i = 0; i < tris.cols(); ++i) {
			const Eigen::Vector3i f = tris.col(i) + Eigen::Vector3i::Ones();
			os << "f " << f.transpose() << "\n";
		}
		os.close();
		return 1;
	}
}