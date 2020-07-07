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
	SurfaceMesh mesh;
	mesh.read(ori_mesh_path);

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
	mesaure.CalcConvexCircumferences(mesh);
}
