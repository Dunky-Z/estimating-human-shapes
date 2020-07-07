#pragma once
#include "pmp/SurfaceMesh.h"
#include <Eigen/Dense>

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

};
