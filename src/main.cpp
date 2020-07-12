//#include "estimate_shape.h"
//#include "reshaper.h"
//
//int main()
//{
//	clock_t t = clock();
//
//	std::vector<std::vector<std::vector<double>>> control_points;
//	Reshaper reshaper;
//	Measure measure;
//	reshaper.SaveBinControlPoint(control_points);
//
//	//保存模型顶点，面片信息
//	//reshaper.SaveVertFacetInBin();
//
//	Eigen::MatrixXd verts; //shape(3V, num_models)
//	Eigen::Matrix3Xi facets;
//
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "vertex").c_str(), verts);
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "facets").c_str(), facets);
//
//
//	//measure.CalcCircumferencesAndSave();
//
//	Eigen::MatrixX3f circum;
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "circumferences").c_str(), circum);
//
//	//测量并保存尺寸信息
//	Eigen::MatrixXd measurelist;
//	//measure.ConvertMeasure(verts, facets, control_points, measurelist, circum);
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "measure_list").c_str(), measurelist);
//
//	//计算并保存平均模型
//	Eigen::Matrix3Xd undeform_mesh;
//	//reshaper.CalcMeanMesh((BIN_DATA_PATH + "AVE.obj").c_str(), verts, facets, undeform_mesh);
//	meshio::ReadObj((BIN_DATA_PATH + "AVE.obj").c_str(), undeform_mesh, facets);
//
//	//读取出来的矩阵形式为（3，VERTS）,所以需要转置一下。也可以改一下GetInverse的代码，但是暂且不管（Todo）
//	Eigen::MatrixX3d undeform_mesh_ = undeform_mesh.transpose();
//	//计算v^{-1}矩阵
//	Eigen::MatrixXd v_inverse;
//	//reshaper.GetKnownUndeformedVerticeInverse(undeform_mesh_, facets, v_inverse);
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "v_inverse").c_str(), v_inverse);
//
//
//	//保存一些基本信息
//	Eigen::MatrixXd Q_transformation, Q_determinant, mean_deform, std_deform;
//	//reshaper.SaveDataInBinary(verts, facets, v_inverse, Q_transformation, Q_determinant, mean_deform, std_deform); 
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "Q_transformation").c_str(), Q_transformation);
//
//	//对Deformation降维，获取人体参数
//	Eigen::MatrixXd coefficient, basis;
//	//reshaper.GetTransformationBasis(Q_transformation, coefficient, basis);
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "coefficient").c_str(), coefficient);
//
//
//	//已知人体参数和尺寸参数求对应的Deformation
//	//Eigen::MatrixXd measure2deform;
//	//reshaper.GetMeasure2Deform(coefficient, measurelist, measure2deform);
//	//binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH +"measure2deform").c_str(), measure2deform);
//
//
//	//构建大型系数矩阵A
//	Eigen::SparseMatrix<double> A; (3 * facets.cols(), 12500);//(3F, VERTS)
//	reshaper.ConstructMatrix(undeform_mesh_, facets, A);
//
//	//输入尺寸信息
//	Eigen::MatrixXd input_measure_norm(15, 1), input_measure(15, 1);
//	input_measure <<
//		//4698.85,
//		1795.61,
//		460.47,
//		//1212.81,
//		//1098.78,
//		//1134.35,
//		890.41,
//		823.41,
//		419.05,
//		824.58,
//		1126.35,
//		1199.55,
//		1336.46,
//		649.92,
//		623.889,
//		204.25,
//		1313.27,
//		442.89,
//		726.47;
//
//	/*----------------------------------*/
//	//调用python脚本，预测尺寸信息
//	py::scoped_interpreter python;
//
//	//py::module py_test = py::module::import("reshaper");
//
//	//py::object result = py_test.attr("predict")(input_measure);
//
//	//Eigen::MatrixXd output_measure = result.cast<Eigen::MatrixXd>();
//	//std::cout << "In c++ \n" << output_measure << std::endl;
//	/*----------------------------------*/
//
//	//采用全局映射法
//	//GlobalMapping(input_measure_norm, A, facets);
//
//	//采用递归消除-局部映射
//	reshaper.RFEMapping(input_measure, A, facets);
//
//	cout << "Main spend : " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds!" << endl;
//	getchar();
//	return 0;
//}
//
