#pragma once
#include <algorithm>
#include <Eigen/Dense>

#include "../geodesic/geodesic_mesh.h"
#include "../geodesic/geodesic_algorithm_base.h"
#include "../geodesic/geodesic_algorithm_exact.h"
#include "../geodesic/geodesic_algorithm_dijkstra.h"
#include "../geodesic/geodesic_algorithm_subdivision.h"

using namespace std;
class measure
{
private:
	static const int N = 12;
	static const int M = 5;
	int lengthKeyPoint[M][2] = {
{12498, 138}/*croth_knee_floorøÁ∏ﬂ*/,
{10814, 10848}/*across_back_shoulder_neckºÁøÌ*/,
{11122, 4486}/*neck_to_gluteal_hip±≥≥§*/,
{10886, 6621}/*shoulder_to_midhand–‰≥§*/,
{8457, 285}/*outer_natural_waist_to_floor—¸∏ﬂ*/
	};

	int circleKeyPoint[N][4] = {
{10963, 11179, 11140, 10943}/*neckæ±Œß*/,
{9285, 9240, 9019, 8666}/*chest–ÿŒß---------*/,
{6937, 7118, 7230, 7124}/*belly_button_waistœ¬—¸Œß-------*/,
{5735, 6275, 6329, 6182}/*gluteal_hip…œÕŒŒß-----------*/,
{7644, 7804, 7724, 7800}/*natural_waist…œ—¸Œß--------------*/,
{4642, 5015, 4828, 4856}/*maximum_hipœ¬ÕŒŒß-------*/,
{9821, 10132, 9942, 9622}/*mid_upper_arm…œ±€Œß+-------*/,
{6640, 6711, 6697, 6574}/*wristÕÛŒß----------*/,
{2645, 2630, 2557, 2622}/*kneeœ•Œß---------*/,
{3930, 3975, 4053, 4008}/*maximum_thigh¥ÛÕ»Œß---------*/,
{11053, 10775, 8219,6034}/*neck_shoulder_elbow_wirst∞Î…ÌøÌ-------------*/,
{8491, 6441, 4122, 8587}/*natural_waist_rise–ÿ∏ﬂ*/,
	};

	double length[M];
	double circle[N];

public:
	string SemanticLable[N + M] =
	{
		"neck", "chest", "belly_button_waist", "gluteal_hip", "natural_waist",
		"maximum_hip", "mid_upper_arm", "wrist", "knee", "maximum_thigh", "neck_shoulder_elbow_wirst","natural_waist_rise",

		"croth_knee_floor", "across_back_shoulder_neck", "neck_to_gluteal_hip", 
		"shoulder_to_midhand", "outer_natural_waist_to_floor"
	};
	measure();
	~measure();

	void initMesh(geodesic::Mesh &mesh, const Eigen::Matrix3Xd & V, Eigen::Matrix3Xi &F);

	void savePath(std::ofstream &out, const std::vector<geodesic::SurfacePoint> &path);

	void calcExact(const Eigen::Matrix3Xd &V, Eigen::Matrix3Xi &F, bool save = true);

	void calcSubdivide(const Eigen::Matrix3Xd &V, Eigen::Matrix3Xi &F);
	void calcDijkstra(const Eigen::Matrix3Xd &V, Eigen::Matrix3Xi &F);
	void calcLength(geodesic::GeodesicAlgorithmBase *algo, geodesic::Mesh &mesh, bool save = false);
	void calcCircle(geodesic::GeodesicAlgorithmBase *algo, geodesic::Mesh &mesh, bool save = false);

	void writeVTK(const std::string &filename, std::vector<geodesic::SurfacePoint> &path);

	void saveParam(Eigen::MatrixXd &Param, int col) {
		for (int i = 0; i < N; ++i)
			Param(i, col) = circle[i];
		for (int i = 0; i < M; ++i)
			Param(i + N, col) = length[i];
	}

	void init() {
		for (int i = 0; i < N; ++i) circle[i] = 0;
		for (int j = 0; j < M; ++j) length[j] = 0;
	}

	void printInfo(int id) {
		if (id < N)
			cout << SemanticLable[id] << " = " << circle[id] << endl;
		else
			cout << SemanticLable[id] << " = " << length[id - N] << endl;
	}
	void printAll() {
		cout << "Print all of the semantic lables.." << endl;
		for (int i = 0; i < N; ++i) {
			cout << SemanticLable[i] << " = " << circle[i] << endl;
		}
		for (int i = 0; i < M; ++i) {
			cout << SemanticLable[i + N] << " = " << length[i] << endl;
		}
	}
	int len() {
		return N + M;
	}

};

