#include "estimate_shape.h"
#include "reshaper.h"

int main()
{
	clock_t t = clock();

	std::vector<std::vector<std::vector<double>>> control_points;
	Reshaper reshaper;
	Measure measure;
	reshaper.SaveBinControlPoint(control_points);

	//����ģ�Ͷ��㣬��Ƭ��Ϣ
	//reshaper.SaveVertFacetInBin();

	Eigen::MatrixXd verts; //shape(3V, num_models)
	Eigen::Matrix3Xi facets;

	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "vertex").c_str(), verts);
	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "facets").c_str(), facets);


	//measure.CalcCircumferencesAndSave();
	Eigen::MatrixX3f circum;
	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "circumferences").c_str(), circum);

	//����������ߴ���Ϣ
	Eigen::MatrixXd measurelist;
	//measure.ConvertMeasure(verts, facets, control_points, measurelist, circum);
	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "measure_list").c_str(), measurelist);

	//���㲢����ƽ��ģ��
	Eigen::Matrix3Xd undeform_mesh;
	//reshaper.CalcMeanMesh((BIN_DATA_PATH + "AVE.obj").c_str(), verts, facets, undeform_mesh);
	meshio::ReadObj((BIN_DATA_PATH + "AVE.obj").c_str(), undeform_mesh, facets);

	//��ȡ�����ľ�����ʽΪ��3��VERTS��,������Ҫת��һ�¡�Ҳ���Ը�һ��GetInverse�Ĵ��룬�������Ҳ��ܣ�Todo��
	Eigen::MatrixX3d undeform_mesh_ = undeform_mesh.transpose();
	//����v^{-1}����
	Eigen::MatrixXd v_inverse;
	//reshaper.GetKnownUndeformedVerticeInverse(undeform_mesh_, facets, v_inverse);
	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "v_inverse").c_str(), v_inverse);


	//����һЩ������Ϣ
	Eigen::MatrixXd Q_transformation, Q_determinant, mean_deform, std_deform;
	//reshaper.SaveDataInBinary(verts, facets, v_inverse, Q_transformation, Q_determinant, mean_deform, std_deform); 
	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "Q_transformation").c_str(), Q_transformation);

	//��Deformation��ά����ȡ�������
	Eigen::MatrixXd coefficient, basis;
	//reshaper.GetTransformationBasis(Q_transformation, coefficient, basis);
	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "coefficient").c_str(), coefficient);


	//��֪��������ͳߴ�������Ӧ��Deformation
	Eigen::MatrixXd measure2deform;
	reshaper.GetMeasure2Deform(coefficient, measurelist, measure2deform);
	//common::read_matrix_binary_from_file((BIN_DATA_PATH +"measure2deform").c_str(), measure2deform);

	cout << "Main spend : " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds!" << endl;
	getchar();
	return 0;
}