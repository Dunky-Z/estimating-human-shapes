#include "measure_t.h"
#include "vtk.h"
#include <iostream>

using namespace std;


Measurement::Measurement() { }

Measurement::~Measurement() { }

// line to *.vtk, visualize the geodesics and check.
void Measurement::writeVTK(const std::string &filename,
	std::vector<geodesic::SurfacePoint> &path)
{
	std::vector<double> nodes;
	std::vector<int> lines;
	std::ofstream os(filename);

	int n = path.size();
	for (int i = 0; i < n; ++i) {
		nodes.push_back(path[i].x());
		nodes.push_back(path[i].y());
		nodes.push_back(path[i].z());

		if (i) {
			lines.push_back(i - 1);
			lines.push_back(i);
		}
	}

	line2vtk(os, nodes.data(), nodes.size() / 3, lines.data(), lines.size() / 2);
	os.close();
	return;
}


void Measurement::initMesh(geodesic::Mesh &mesh,
	const Eigen::Matrix3Xd & V, Eigen::Matrix3Xi & F)
{
	init();
	vector<unsigned> ff;

	for (int i = 0; i < F.cols(); ++i) {
		if (i == 5014 || i == 24932) continue;
		for (int j = 0; j < F.rows(); ++j) {
			ff.push_back(F(j, i));
		}
	}
	vector<double> vv(V.data(), V.data() + V.rows()*V.cols());

	mesh.initialize_mesh_data(vv, ff);

	cout << "Mesh initialized ..." << endl;
	return;
}



void Measurement::calcExact(const Eigen::Matrix3Xd & V,
	Eigen::Matrix3Xi & F, bool save)
{
	geodesic::Mesh mesh;
	initMesh(mesh, V, F);
	geodesic::GeodesicAlgorithmExact algo(&mesh);
	cout << "save ? = " << save << endl;
	calcLength(&algo, mesh, save);
	calcCircle(&algo, mesh, save);
	printAll();
}

void Measurement::calcSubdivide(const Eigen::Matrix3Xd & V,
	Eigen::Matrix3Xi & F)
{
	geodesic::Mesh mesh;
	initMesh(mesh, V, F);

	geodesic::GeodesicAlgorithmSubdivision algo(&mesh, 2);

	calcLength(&algo, mesh);

	calcCircle(&algo, mesh);

	printAll();
}

void Measurement::CalcGeodesicAndCircum(
	const Eigen::Matrix3Xd & V,
	Eigen::Matrix3Xi & F,
	bool save)
{
	geodesic::Mesh mesh;
	initMesh(mesh, V, F);
	geodesic::GeodesicAlgorithmDijkstra algo_1(&mesh);
	calcLength(&algo_1, mesh, save);

	initMesh(mesh, V, F);
	geodesic::GeodesicAlgorithmExact algo_2(&mesh);
	calcCircle(&algo_2, mesh, save);

	printAll();
}

void Measurement::calcDijkstra(const Eigen::Matrix3Xd & V,
	Eigen::Matrix3Xi &F)
{
	geodesic::Mesh mesh;
	initMesh(mesh, V, F);

	geodesic::GeodesicAlgorithmDijkstra algo(&mesh);

	calcLength(&algo, mesh);

	calcCircle(&algo, mesh);

	printAll();
}

void Measurement::savePath(std::ofstream &out, const std::vector<geodesic::SurfacePoint> &path)
{
	for (auto p : path) {
		cout << p.type() << " - ";
		int x, y;
		double t;
		if (p.type() == geodesic::VERTEX)
		{
			geodesic::vertex_pointer v = static_cast<geodesic::vertex_pointer>(p.base_element());
			x = v->id(), y = 0;
			t = 0;
			cout << v->id() << " 0" << " 0";
		}
		else if (p.type() == geodesic::EDGE) {
			geodesic::edge_pointer e = static_cast<geodesic::edge_pointer>(p.base_element());
			x = e->v0()->id(), y = e->v1()->id();
			t = p.distance(e->v0()) / e->length();
			cout << e->v0()->id() << " ";
			cout << e->v1()->id() << " ";
			cout << p.distance(e->v0()) / e->length();
		}
		else {
			cout << "wrong point type" << p.type() << endl;
		}
		out.write((const char*)(&x), sizeof(int));
		out.write((const char*)(&y), sizeof(int));
		out.write((const char*)(&t), sizeof(double));
		cout << endl;
	}
}

void Measurement::calcLength(geodesic::GeodesicAlgorithmBase *algo,
	geodesic::Mesh &mesh, bool save)
{
	/*******************Calculate Geodesic Length*******************/
	ofstream length_out("../data/path/length.txt");
	for (int i = 0; i < M; ++i) {
		//length_out.write((const char*)(&M), sizeof(double));
		cout << "now " << SemanticLable[i] << endl;
		int s = lengthKeyPoint[i][0];
		int t = lengthKeyPoint[i][1];
		std::vector<geodesic::SurfacePoint> source;
		source.push_back(geodesic::SurfacePoint(&mesh.vertices()[s]));

		geodesic::SurfacePoint target(&mesh.vertices()[t]);

		algo->propagate(source);
		//algo->print_statistics();

		std::vector<geodesic::SurfacePoint> path;
		algo->trace_back(target, path);
		geodesic::print_info_about_path(path);
		length[i] = geodesic::length(path);
		std::cout << length[i] << std::endl;
		int len = path.size();
		if (save)
		{
			ofstream out("../data/path/" + SemanticLable[i] + "0", ios::binary);
			out.write((const char*)(&len), sizeof(int));

			savePath(out, path);
			string name = "../data/measure_vtk/" + to_string(i) + "_" + SemanticLable[i] + ".vtk";
			writeVTK(name, path);
			//printInfo(N + i);
			out.close();
		}
		length_out << length[i] << std::endl;
	}
	length_out.close();
}


void Measurement::calcCircle(geodesic::GeodesicAlgorithmBase *algo,
	geodesic::Mesh &mesh, bool save)
{
	/*********************Calculate Circumstance********************/
	ofstream circle_out("../data/path/circle.txt");
	for (int i = 0; i < N; ++i) {
		//circle_out.write((const char*)(&N), sizeof(double));
		cout << "now " << SemanticLable[M + i] << endl;
		std::vector<geodesic::SurfacePoint> path[4];
		int len = 0;
		for (int j = 0; j < 4; ++j)
		{
			int s = circleKeyPoint[i][j];
			int t = circleKeyPoint[i][(j + 1) % 4];

			std::vector<geodesic::SurfacePoint> source;
			source.push_back(geodesic::SurfacePoint(&mesh.vertices()[s]));

			geodesic::SurfacePoint target(&mesh.vertices()[t]);

			algo->propagate(source);
			//algo->print_statistics();

			algo->trace_back(target, path[j]);
			geodesic::print_info_about_path(path[j]);
			circle[i] += geodesic::length(path[j]);

			len += path[j].size();
			if (save)
			{
				ofstream out("../data/path/" + SemanticLable[M + i] + to_string(j), ios::binary);
				out.write((const char*)(&len), sizeof(int));
				savePath(out, path[j]);

				string name = "../data/measure_vtk/" + to_string(i) + "-" + to_string(j) + "-" + SemanticLable[M + i] + ".vtk";
				writeVTK(name, path[j]);
				out.close();
			}
		}
		circle_out << circle[i] << std::endl;
		//printInfo(i);
	}
	circle_out.close();
}

