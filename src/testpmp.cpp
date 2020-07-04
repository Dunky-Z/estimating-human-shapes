////#include <iostream>
//#include "pmp/SurfaceMesh.h"
//#include <Eigen/Dense>
//
//using namespace Eigen;
//using namespace pmp;
//using namespace std;
//using Point = pmp::Point;
//
//int main() 
//{
//	SurfaceMesh mesh;
//	Vertex v0, v1, v2, v3;
//	v0 = mesh.add_vertex(Point(0, 0, 0));
//	v1 = mesh.add_vertex(Point(1, 0, 0));
//	v2 = mesh.add_vertex(Point(0, 1, 0));
//	v3 = mesh.add_vertex(Point(0, 0, 1));
//	mesh.add_triangle(v0, v1, v3);
//	mesh.add_triangle(v1, v2, v3);
//	mesh.add_triangle(v2, v0, v3);
//	mesh.add_triangle(v0, v2, v1);
//	cout << "vertices: " << mesh.n_vertices() << std::endl;
//	cout << "edges: " << mesh.n_edges() << std::endl;
//	cout << "faces: " << mesh.n_faces() << std::endl;
//
//	return 0;
//}
//
