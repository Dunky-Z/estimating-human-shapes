//#include "reshaper.h"
//
//
//Reshaper::Reshaper() {}
//
//Reshaper::~Reshaper() {}
//
//
///*!
//*@brief  读取控制点文件，并将控制点保存进数组
//*@param[out]
//*@param[in]  std::vector<std::vector<std::vector<double>>> & control_points  保存控制点的数组
//*@return     void
//*/void Reshaper::SaveBinControlPoint(std::vector<std::vector<std::vector<double>>>& control_points)
//{
//	std::cout << "Begin load cp..." << std::endl;
//	std::ifstream is("./data/body_control_points.txt");
//	assert(is);
//
//	std::vector<std::vector<double>> templist;
//	std::vector<double> tempNode;
//	std::string line;
//	double t_num;
//	while (!is.eof())
//	{
//		std::getline(is, line);
//		if (line.empty())
//			continue;
//
//		if (line[0] == '#')
//		{
//			//cout << "每个尺寸有几个点： " << templist.size() << endl;
//			if (templist.size() != 0)
//			{
//				control_points.push_back(templist);
//				templist.clear();
//			}
//		}
//		else if (line.find(" ") == std::string::npos)
//		{
//			continue;
//		}
//		else
//		{
//			std::istringstream instream(line);
//			//将输入流以空格为分割 存入一个数组
//			while (instream >> t_num)
//			{
//				//cout << t_num << "  ";
//				tempNode.push_back(t_num);
//			}
//			//cout << "每个行有几个元素 ： "<< tempNode.size() << endl;
//			templist.push_back(tempNode);
//			tempNode.clear();
//		}
//	}
//	//cout << "每个尺寸有几个点： " << templist.size() << endl;
//	control_points.push_back(templist);
//	std::cout << "Load control_points done!" << std::endl;
//}
//
///*!
//*@brief			保存每个尺寸由哪些边构成
//*@param[out]
//*@param[in]  std::vector<std::vector<std::vector<double>>> & control_points  [尺寸个数，每个尺寸包含的控制点，每个控制点]
//*@param[in]  const char * filename
//*@param[in]  const std::string & path
//*@param[in]  const std::vector<std::string> & files [19个尺寸相关的顶点]
//*@return     void
//*/
//void Reshaper::SaveBinEdge(std::vector<std::vector<std::vector<double>>>& control_points, std::vector<std::vector<int>> &point_idx)
//{
//	Eigen::Matrix2Xi edge;
//	//因为controlpoints数组中有重复的控制点信息，所以要去重，剩下来的顶点就可串成线，方便后续计算
//	for (auto &measure : control_points)
//	{
//		std::vector<int> point_idx_;
//		for (int i = 0; i < measure.size(); ++i)
//		{
//			if (measure[i][0] == 1)
//			{
//				//在顶点数组中查找是否已经存在该顶点，如果存在就跳过
//				std::vector<int>::iterator it = std::find(point_idx_.begin(), point_idx_.end(), measure[i][1]);
//				if (it == point_idx_.end())
//				{
//					point_idx_.push_back(measure[i][1]);
//				}
//			}
//			else if (measure[i][0] == 2)
//			{
//				auto it1 = std::find(point_idx_.begin(), point_idx_.end(), measure[i][1]);
//				if (it1 == point_idx_.end())
//				{
//					point_idx_.push_back(measure[i][1]);
//				}
//				auto it2 = std::find(point_idx_.begin(), point_idx_.end(), measure[i][2]);
//				if (it2 == point_idx_.end())
//				{
//					point_idx_.push_back(measure[i][2]);
//				}
//			}
//			else
//			{
//				std::vector<int>::iterator it1 = std::find(point_idx_.begin(), point_idx_.end(), measure[i][1]);
//				if (it1 == point_idx_.end())
//				{
//					point_idx_.push_back(measure[i][1]);
//				}
//				std::vector<int>::iterator it2 = std::find(point_idx_.begin(), point_idx_.end(), measure[i][2]);
//				if (it2 == point_idx_.end())
//				{
//					point_idx_.push_back(measure[i][2]);
//				}
//				std::vector<int>::iterator it3 = std::find(point_idx_.begin(), point_idx_.end(), measure[i][3]);
//				if (it3 == point_idx_.end())
//				{
//					point_idx_.push_back(measure[i][3]);
//				}
//			}
//		}
//		point_idx.push_back(point_idx_);
//		SavePointToFile(points_idx_path, point_idx);
//	}
//}
//
////void Reshaper::FitOneMeasurements(Eigen::Matrix3Xd & res, std::vector<int> point_idx, const Eigen::Matrix3Xd & vertices, const double measurement)
////{
////
////	int num_edge;
////	if (num_v > 2)
////		num_edge = num_v;
////	else
////		num_edge = 1;
////
////	Eigen::SparseMatrix<double> A;
////	Eigen::VectorXd b;
////	typedef Eigen::Triplet<double> Tri;
////	std::vector<Tri> triplets;
////
////	A.resize(3 * num_edge, 3 * num_v);
////	b.setConstant(3 * num_edge, 0);
////
////	for (int j = 0; j < num_measure; ++j)
////	{
////		for (int i = 0; i < 3 * num_edge; i = i + 3)
////		{
////			const int edge_0 = point_idx[i % (3 * num_edge)];
////			const int edge_1 = point_idx[(i + 1) % (3 * num_edge)];
////			const Eigen::Vector3d& v0 = vertices.col(edge_0);
////			const Eigen::Vector3d& v1 = vertices.col(edge_1);
////			const Eigen::Vector3d& edge_01 = v1 - v0;
////			const double edge_len = edge_01.norm();
////
////			for (int j = 0; j < 3; ++j)
////			{
////				triplets.push_back(Tri(i + j, edge_0 * 3, -1));
////				triplets.push_back(Tri(i + j, edge_1 * 3, 1));
////				b(i + j) = ((edge_01[j] / edge_len)*(measurement / num_edge));
////			}
////		}
////	}
////
////
////
////	A.setFromTriplets(triplets.begin(), triplets.end());
////	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
////	solver.compute(A.transpose() * A);
////	Eigen::VectorXd vecV = solver.solve(A.transpose() * b);
////	res = Eigen::Map<Eigen::Matrix3Xd>(vecV.data(), 3, num_v);
////}
//
//void Reshaper::FitMeasurements(Eigen::Matrix3Xd& res_verts, std::vector<std::vector<int>> point_idx, const Eigen::Matrix3Xd &vertices, const Eigen::MatrixXd measurements)
//{
//	const int num_measure = point_idx.size()-1;
//	//保存每个尺寸的边数量
//	std::vector<int> edge;
//	for (auto& num_v : point_idx)
//	{
//		if (num_v.size() > 2)
//		{
//			edge.push_back(num_v.size());
//		}
//		else
//		{
//			//edge.push_back(1);
//			continue;
//		}
//	}
//	//所有边的数量
//	int num_edge_all = std::accumulate(edge.begin(), edge.end(), 0);
//
//	typedef Eigen::Triplet<double> Tri;
//	std::vector<Tri> triplets;
//	triplets.reserve(6 * num_edge_all);
//	Eigen::VectorXd b;
//	b.setConstant(3 * num_edge_all, 0);
//	Eigen::SparseMatrix<double> A;
//	A.resize(3 * num_edge_all, 3 * vertices.cols());
//
//	int row = 0;
//	for (int i = 0; i < num_measure; ++i)
//	{
//
//		for (int j = 0; j < edge[i]; ++j)
//		{
//			const int edge_0 = point_idx[i+1][j % edge[i]];
//			const int edge_1 = point_idx[i+1][(j + 1) % edge[i]];
//			const Eigen::Vector3d v0 = vertices.col(edge_0);
//			const Eigen::Vector3d v1 = vertices.col(edge_1);
//			const Eigen::Vector3d edge_01 = v1 - v0;
//			const double edge_len = edge_01.norm();
//			const Eigen::Vector3d auxd = (edge_01 / edge_len) * (measurements(i+1, 0) / edge[i]);
//
//			for (int k = 0; k < 3; ++k)
//			{
//				triplets.push_back(Tri(row + j * 3 + k, edge_0 * 3 + k, -1));
//				triplets.push_back(Tri(row + j * 3 + k, edge_1 * 3 + k, 1));
//				b(row + j * 3 + k) = auxd[k];
//				std::cout << row + j * 3 + k << " " << edge_0 * 3 + k << " " << auxd[k] << std::endl;
//			}
//		}
//		row += edge[i] * 3;
//	}
//
//	A.setFromTriplets(triplets.begin(), triplets.end());	
//	auto AT = A.transpose();
//	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
//	//solver.compute(A * AT);
//	solver.compute(A.transpose() * A);
//	if (solver.info() != Eigen::Success)
//		std::cout << "solve failed !" << std::endl;
//	Eigen::VectorXd vecV = solver.solve(A.transpose() * b);
//	//Eigen::VectorXd vecV = AT *  solver.solve(b);
//	res_verts = Eigen::Map<Eigen::Matrix3Xd>(vecV.data(), 3, vertices.cols());
//}
///*!
//*@brief  以二进制保存顶点信息
//*@param[out]
//*@param[in]  const char * filename  保存的文件名
//*@param[in]  const std::string & path  模型数据集路径
//*@param[in]  const std::vector<std::string> & files  模型数据文件名
//*@return     void
//*/void Reshaper::SaveBinVerts(const char * filename, const std::string & path, const std::vector<std::string>& files)
//{
//	Eigen::MatrixXd vertices;
//	vertices.resize(VERTS * 3, files.size());
//	//cout << "vertices.shape = " << vertices.rows() << " " << vertices.cols() << endl;
//	int k = 0;
//	for (auto file : files)
//	{
//		Eigen::Matrix3Xd vv;
//		Eigen::Matrix3Xi ff;
//		meshio::ReadObj(path + file, vv, ff);
//		int cnt = 0;
//		//std::cout << vv.cols() << " " << vv.rows() << std::endl;
//		// be careful with the cols and rows
//		for (int i = 0; i < vv.cols(); ++i)
//		{
//			for (int j = 0; j < vv.rows(); ++j)
//			{
//				vertices(cnt++, k) = vv(j, i);
//			}
//		}
//		//cout << "cnt = " << file << endl;
//		k++;
//	}
//	binaryio::WriteMatrixBinaryToFile(filename, vertices);
//	//cout << "ok" << endl;
//}
//
///*!
//*@brief  以二进制保存面片信息
//*@param[out]
//*@param[in]  const char * filename  保存的文件名
//*@param[in]  const std::string & path  模型数据集路径
//*@param[in]  const std::vector<std::string> & files  模型文件名
//*@return     void
//*/void Reshaper::SaveBinFaces(const char * filename, const std::string & path, const std::vector<std::string>& files)
//{
//	Eigen::Matrix3Xd vv;
//	Eigen::Matrix3Xi ff;
//	meshio::ReadObj(path + files[0], vv, ff);
//	binaryio::WriteMatrixBinaryToFile(filename, ff);
//	std::cout << "Save facets done!" << std::endl;
//}
//
///*!
//*@brief  以二进制保存所有模型的顶点信息和面片信息
//*@param[out]
//*@return     void
//*/void Reshaper::SaveVertFacetInBin()
//{
//	std::string trainModelPath = DATASET_PATH;
//	std::vector<std::string> trainFiles = GetFiles(trainModelPath + "*");
//
//	SaveBinVerts((BIN_DATA_PATH + "vertex").c_str(), trainModelPath, trainFiles);
//
//	// F (3, 12500)
//	SaveBinFaces((BIN_DATA_PATH + "facets").c_str(), trainModelPath, trainFiles);
//
//	Eigen::MatrixXd verts;
//	Eigen::Matrix3Xi facets;
//
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "vertex").c_str(), verts);
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "facets").c_str(), facets);
//
//	std::cout << facets.cols() << std::endl;
//	std::cout << "Save facets verts done!" << std::endl;
//}
//
//
//void Reshaper::CalcMeanMesh(const char * filename, const Eigen::MatrixXd &vertices, const Eigen::Matrix3Xi &facets, Eigen::Matrix3Xd &undeform_mesh)
//{
//	cout << "Begin CalcMeanMesh..." << endl;
//	Eigen::MatrixXd newV = vertices.rowwise().mean();
//	newV.resize(3, VERTS);
//	meshio::SaveObj(filename, newV, facets);
//	cout << "Finished CalcMeanMesh!" << endl;
//}
//
//void Reshaper::GetKnownUndeformedVerticeInverse(Eigen::MatrixX3d &undeform_mesh, const Eigen::Matrix3Xi &facets, Eigen::MatrixXd &v_inverse)
//{
//	cout << "Start Calc Inverse..." << endl;
//	clock_t t = clock();
//	v_inverse.resize(3 * FACES, 3);
//	// i为三角面片的下标
//	for (int i = 0; i < FACES; ++i)
//	{
//		Eigen::Vector3d v_undeformed[4];
//		for (int j = 0; j < 3; ++j)
//		{
//			//idx为顶点的下标
//			int idx_undeformed[3];
//			idx_undeformed[j] = facets(j, i);
//
//			//k 表示顶点的x,y,z坐标
//			for (int k = 0; k < 3; ++k)
//			{
//				v_undeformed[j](k) = undeform_mesh(idx_undeformed[j], k);
//			}
//		}
//
//		//虚构第四个顶点
//		Eigen::Vector3d edge1_underformed = v_undeformed[1] - v_undeformed[0];
//		Eigen::Vector3d edge2_underformed = v_undeformed[2] - v_undeformed[0];
//		Eigen::Vector3d edge3_underformed = edge1_underformed.cross(edge2_underformed);
//		double edge3_underformed_norm = edge3_underformed.norm();
//		Eigen::Vector3d v_cross = edge3_underformed.array() / std::sqrt(edge3_underformed_norm);
//
//		v_undeformed[3] = v_undeformed[0] + v_cross;
//
//		//建立边组成的矩阵
//		//V_j = [v_i2 - v_i1	v_i3 - v_i1  v_i4 - v_i1]
//		Eigen::Matrix3d v_edge_undeformed;
//		for (int i = 0; i < 3; ++i)
//		{
//			v_edge_undeformed.col(i) = v_undeformed[i + 1] - v_undeformed[0];
//		}
//		Eigen::Matrix3d v_inverse_ = v_edge_undeformed.inverse();
//		v_inverse.block<3, 3>(i * 3, 0) = v_inverse_;
//	}
//
//	//printShape(v_inverse);
//	//cout << v_inverse.topRows(9) << endl;
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "v_inverse").c_str(), v_inverse);
//	cout << "Calculate Inverse spend: " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds." << endl;
//}
//
//
//void Reshaper::SaveDataInBinary(Eigen::MatrixXd &vertices, const Eigen::Matrix3Xi &facets, Eigen::MatrixXd v_inverse,
//	Eigen::MatrixXd &Q_transformation, Eigen::MatrixXd &Q_determinant, Eigen::MatrixXd &mean_deform, Eigen::MatrixXd &std_deform)
//{
//	cout << "Start Save data..." << endl;
//	clock_t t = clock();
//	int num_models = vertices.cols();
//	Q_transformation.resize(num_models, FACES * 9);
//	Q_determinant.resize(num_models, FACES);
//	Eigen::MatrixXd mesh, t_mesh;
//
//
//	//Eigen::MatrixXd shu;
//	//shu.resize(6, 3);
//	//shu << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18;
//	//cout << shu << endl;
//	//shu.resize(1, 18);
//	//cout << shu << endl;
//
//	for (int i = 0; i < num_models; ++i)
//	{
//		//取出第i个模型的顶点信息
//		mesh.resize(VERTS, 3);
//		t_mesh = vertices.col(i);
//
//		for (int p = 0; p < VERTS; ++p)
//		{
//			Eigen::MatrixXd t_v = t_mesh.block<3, 1>(3 * p, 0);
//			t_v.transposeInPlace();
//			mesh.row(p) = t_v;
//		}
//
//		Eigen::MatrixXd v_hat;
//		Eigen::MatrixXd q_deformation;
//		Eigen::MatrixXd q_determinant;
//		q_determinant.resize(1, FACES);
//		q_determinant.setZero();
//		q_deformation.resize(3 * FACES, 3);
//		v_hat = GetOneModelTransformation(mesh, facets);//shape(3*F,3)
//
//		//论文公式（4）
//		Eigen::Matrix3d deformation_f, inverse_f, deformation;
//		for (int i = 0; i < FACES; ++i)
//		{
//			deformation_f = v_hat.block<3, 3>(3 * i, 0);
//			inverse_f = v_inverse.block<3, 3>(3 * i, 0);
//			deformation = deformation_f * inverse_f;
//			q_deformation.block<3, 3>(3 * i, 0) = deformation;
//			q_determinant(0, i) = deformation.determinant();
//		}
//
//		//cout << "q_deformation: " << endl;
//		//cout << q_deformation.topRows(9) << endl;
//
//		q_deformation.transposeInPlace();
//		q_deformation.resize(1, 9 * FACES);
//
//		//cout << "q_deformation---------" << endl;
//		//cout << q_deformation.leftCols(90) << endl;
//
//		Q_transformation.row(i) = q_deformation;
//		Q_determinant.row(i) = q_determinant;
//	}
//	//cout << Q_transformation.leftCols(9) << endl;
//	//cout << Q_determinant.leftCols(1) << endl;
//	//cout << "dd" << endl;
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "Q_determinant").c_str(), Q_determinant);
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "Q_transformation").c_str(), Q_transformation);
//
//	cout << Q_transformation.row(0).leftCols(9) << endl;
//
//	mean_deform.resize(1, FACES * 9);
//	std_deform.resize(1, FACES * 9);
//	for (int i = 0; i < FACES * 9; ++i)
//	{
//		//Q_trans :shape（num_models, 9*FACES）.
//		mean_deform(0, i) = Q_transformation.col(i).mean();
//		std_deform(0, i) = CalcStd(Q_transformation.col(i), mean_deform(0, i));
//	}
//
//	//cout << "mean_deform.leftCols(9)" << endl;
//	//cout << mean_deform.leftCols(9) << endl;
//	//mean_deform.resize(3 * FACES, 3);
//	//cout << mean_deform.topRows(6) << endl;
//	//std_deform.resize(3 * FACES, 3);
//	//cout << std_deform.topRows(6) << endl;
//
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "mean_deform").c_str(), mean_deform);
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "std_deform").c_str(), std_deform);
//
//	cout << "Save data spend: " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds." << endl;
//}
//Eigen::MatrixXd Reshaper::GetOneModelTransformation(Eigen::MatrixXd &deformed_mesh, const Eigen::Matrix3Xi &facets)
//{
//	//cout << "Start Calc OneModelTransAndDets..." << endl;
//	clock_t t = clock();
//
//	Eigen::MatrixXd v_edge_deformed(3 * FACES, 3);
//	// i为三角面片的下标
//	for (int i = 0; i < FACES; ++i)
//	{
//		Eigen::Vector3d v_deformed[4];
//		for (int j = 0; j < 3; ++j)
//		{
//			//idx为顶点的下标
//			int idx_deformed[3];
//			idx_deformed[j] = facets(j, i);
//
//			//k 表示顶点的x,y,z坐标
//			for (int k = 0; k < 3; ++k)
//			{
//				v_deformed[j](k) = deformed_mesh(idx_deformed[j], k);
//			}
//		}
//
//		//虚构第四个顶点，论文的公式（1）
//		Eigen::Vector3d edge1_derformed = v_deformed[1] - v_deformed[0];
//		Eigen::Vector3d edge2_derformed = v_deformed[2] - v_deformed[0];
//		Eigen::Vector3d edge3_derformed = edge1_derformed.cross(edge2_derformed);
//		double edge3_derformed_norm = edge3_derformed.norm();
//		Eigen::Vector3d v_cross = edge3_derformed.array() / std::sqrt(edge3_derformed_norm);
//		v_deformed[3] = v_deformed[0] + v_cross;
//
//		//建立边组成的矩阵，论文的公式（3）
//		//V_j = [v_i2 - v_i1	v_i3 - v_i1  v_i4 - v_i1]
//		Eigen::Matrix3d v_edge_deformed_;
//		for (int i = 0; i < 3; ++i)
//		{
//			v_edge_deformed_.col(i) = v_deformed[i + 1] - v_deformed[0];
//		}
//		//分块保存
//		v_edge_deformed.block<3, 3>(i * 3, 0) = v_edge_deformed_;
//	}
//	//cout << "Calculate TransAndDets spend: " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds." << endl;
//	//cout << "v_edge_deformed:   -----" << endl;
//	//cout << v_edge_deformed.topRows(30) << endl;
//	return v_edge_deformed;
//}
//
//double Reshaper::CalcVariance(const Eigen::MatrixXd &x, const double average)
//{
//	double sum = 0.0;
//	int len = x.rows();
//	for (int i = 0; i < len; ++i)
//	{
//		sum += pow(x(i, 0) - average, 2);
//	}
//	return sum / len;
//}
//
//double Reshaper::CalcStd(const Eigen::MatrixXd &x, const double average)
//{
//	double variance = CalcVariance(x, average);
//	return sqrt(variance);
//}
//
//void Reshaper::GetTransformationBasis(Eigen::MatrixXd &transformation, Eigen::MatrixXd &coefficient, Eigen::MatrixXd &basis)
//{
//	cout << "Start GetTransBasis ..." << endl;
//	clock_t t = clock();
//
//	int numModels = transformation.rows();
//	Eigen::MatrixXd mean_deform, std_deform;
//
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "mean_deform").c_str(), mean_deform);
//	binaryio::ReadMatrixBinaryFromFile((BIN_DATA_PATH + "std_deform").c_str(), std_deform);
//
//	//mean_deform.resize(FACES, 9);
//	//cout << mean_deform.topRows(9) << endl;
//
//	//cout << mean_deform.leftCols(9) << endl;
//	//cout << std_deform.leftCols(9) << endl;
//
//	Eigen::MatrixXd temp = transformation.row(0);
//	//cout << "temp.leftCols(9)" << endl;
//	//cout << temp.leftCols(90) << endl;
//	for (int i = 0; i < numModels; ++i)
//	{
//		transformation.row(i) = transformation.row(i) - mean_deform;
//		transformation.row(i) = transformation.row(i).cwiseQuotient(std_deform);
//		//for (int j = 0; j < transformation.cols(); ++j)
//		//{
//		//	transformation(i, j) /= std_deform(0, j);
//		//}
//	}
//
//	//temp.resize(3 * FACES, 3);
//	//cout << "temp.topRows(9)" << endl;
//	//cout << temp.topRows(9) << endl;
//
//	//cout << "transformation.leftCols(9)" << endl;
//	//cout << transformation.leftCols(9) << endl;
//
//	Eigen::MatrixXd d = transformation.transpose();	//shape(9F,model_num)
//
//	//cout << "d.topRows(9)" << endl;
//	//cout << d.topRows(90) << endl;
//
//	JacobiSVD<MatrixXd> svd(d, ComputeThinU | ComputeThinV);
//	Eigen::MatrixXd d_basis_t = svd.matrixU();
//
//	basis = d_basis_t.leftCols(BASIS_NUM);
//	//Eigen::VectorXd sign(10);
//	//sign << -1, -1, -1, -1, -1, -1, 1, -1, 1, -1;
//
//	//for (int i = 0; i < 9 * FACES; ++i)
//	//{
//	//	basis.row(i) = (basis.row(i)).cwiseProduct(sign.transpose().eval());
//	//}
//
//	//cout << "basis.topRows(9)" << endl;
//	//cout << basis.topRows(9) << endl;
//
//	basis.resize(9 * FACES, BASIS_NUM);
//
//	//cout << basis.topRows(9) << endl;
//	Eigen::MatrixXd d_basisT = basis.transpose();
//	//cout << d_basisT.leftCols(9) << endl;
//
//	coefficient = d_basisT * d;
//	//cout << coefficient << endl;
//	//shape(10, 20)
//	//printShape(coefficient);
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "basis").c_str(), basis);
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "coefficient").c_str(), coefficient);
//	cout << "GetTransBasis spend: " << (double)(clock() - t) / CLOCKS_PER_SEC << "seconds." << endl;
//}
//
//
//void Reshaper::GetMeasure2Deform(Eigen::MatrixXd &coefficient, Eigen::MatrixXd &measurelist, Eigen::MatrixXd &measure2deform)
//{
//	int num_measure = measurelist.rows();
//	int num_models = measurelist.cols();
//	Eigen::SparseMatrix<double> M(BASIS_NUM * num_models, BASIS_NUM * num_measure);
//
//	//直接resize会导致按列先序
//	coefficient.transposeInPlace();
//	coefficient.resize(coefficient.size(), 1);
//
//	Eigen::MatrixXd d = coefficient;
//	typedef Triplet<double> Tri;
//	std::vector<Tri> triplets;
//	for (int i = 0; i < num_models; ++i)
//	{
//		for (int j = 0; j < BASIS_NUM; ++j)
//		{
//			for (int k = 0; k < M_NUM; ++k)
//			{
//				triplets.push_back((Tri(BASIS_NUM * i + j, j * M_NUM + k, measurelist.coeff(k, i))));
//			}
//		}
//	}
//	M.setFromTriplets(triplets.begin(), triplets.end());
//
//	SparseLU<Eigen::SparseMatrix<double>> lu((M.transpose())*M);
//	if (lu.info() == Eigen::Success)
//	{
//		measure2deform = lu.solve((M.transpose())*(d));
//	}
//
//	measure2deform.resize(num_measure, BASIS_NUM);
//	measure2deform.transposeInPlace();
//
//	cout << measure2deform << endl;
//	binaryio::WriteMatrixBinaryToFile((BIN_DATA_PATH + "measure2deform").c_str(), measure2deform);
//
//	//cout << ans << endl;
//	//printShape(ans);
//}
//
//void Reshaper::ConstructMatrix(const Eigen::MatrixXd &undeform_mesh_, const Eigen::Matrix3Xi &facets, Eigen::SparseMatrix<double> &A)
//{
//	A.resize(3 * FACES, VERTS);
//	Eigen::Vector3d V_undeformed_target[3];
//	typedef Triplet<double> Tri;
//	std::vector<Tri> triplets;
//
//	for (int j = 0; j < FACES; ++j)
//	{
//		for (int i = 0; i < 3; ++i)
//		{
//			//idx为顶点的下标
//			int index[3];
//			index[i] = facets(i, j);
//
//			//k 表示顶点的x,y,z坐标
//			for (int k = 0; k < 3; ++k)
//			{
//				V_undeformed_target[i](k) = undeform_mesh_(index[i], k);
//			}
//		}
//
//		Eigen::MatrixXd w(3, 2);
//		for (int i = 0; i < 2; ++i)
//		{
//			w.col(i) = V_undeformed_target[i + 1] - V_undeformed_target[0];
//		}
//
//		//文档公式（7）计算QR分解
//		Eigen::MatrixXd Q(3, 3);	//正交矩阵
//		Eigen::MatrixXd R(3, 2);	//上三角矩阵
//		R.setZero();
//		Eigen::MatrixXd Q_block(3, 2);
//		Eigen::MatrixXd R_block(2, 2);
//		QRFactorize(w, Q, R);
//		Q_block = Q.block(0, 0, 3, 2);
//		R_block = R.block(0, 0, 2, 2);
//
//		//文档公式（8）
//		Eigen::MatrixXd T(2, 3);
//		T = R_block.inverse()*(Q_block.transpose());
//
//		int index_target[3];	//顶点下标
//		for (int i = 0; i < 3; ++i)
//		{
//			index_target[i] = facets(i, j);
//		}
//
//		for (int i = 0; i < 3; ++i)
//		{
//			triplets.push_back(Tri(3 * j + i, index_target[1], T.coeff(0, i)));
//			triplets.push_back(Tri(3 * j + i, index_target[2], T.coeff(1, i)));
//			triplets.push_back(Tri(3 * j + i, index_target[0], -T.coeff(0, i) - T.coeff(1, i)));
//		}
//	}
//	A.setFromTriplets(triplets.begin(), triplets.end());
//	//common::write_matrix_binary_to_file("./data/train/A", A);
//}
//
//void Reshaper::QRFactorize(const Eigen::MatrixXd &a, Eigen::MatrixXd &q, Eigen::MatrixXd &r)
//{
//	int i, j, imax, jmax;
//	imax = a.rows();
//	jmax = a.cols();
//
//	for (j = 0; j < jmax; j++)
//	{
//		Eigen::VectorXd v(a.col(j));
//		for (i = 0; i < j; i++)
//		{
//			Eigen::VectorXd qi(q.col(i));
//			r(i, j) = qi.dot(v);
//			v = v - r(i, j)*qi;
//		}
//		float vv = (float)v.squaredNorm();
//		float vLen = sqrtf(vv);
//		if (vLen < EPSILON)
//		{
//			r(j, j) = 1;
//			q.col(j).setZero();
//		}
//		else
//		{
//			r(j, j) = vLen;
//			q.col(j) = v / vLen;
//		}
//	}
//}
//
//void Reshaper::Synthesize(Eigen::SparseMatrix<double> A, Eigen::MatrixXd deform, Eigen::Matrix3Xi &facets)
//{
//	cout << "Start Synthesize..." << endl;
//	//deform.resize(deform.size(), 1);
//	//printShape(deform);//(225000,1)(9F,1)
//	Eigen::MatrixXd X(VERTS, 3);
//
//	//printShape(A); //(75000,125000)
//	//cout << A.block(0, 0, 1000, 100) << endl;
//	Eigen::SparseMatrix<double> t = A.transpose()*A;
//	//printShape(t);
//	//cout << t.block(0, 0, 100, 100) << endl;
//	Eigen::SparseLU<Eigen::SparseMatrix<double>> lu(t);
//
//	X.col(0) = lu.solve(A.transpose()*(deform.col(0)));
//	X.col(1) = lu.solve(A.transpose()*(deform.col(1)));
//	X.col(2) = lu.solve(A.transpose()*(deform.col(2)));
//
//	X.transposeInPlace();
//
//	//将文件名改为当前时间，每次运行不用改文件名，方法比较低级，还需要改进TODO
//	time_t now_time = time(NULL);
//	std::string file_name = string(asctime(localtime(&now_time)));
//	//获取的时间Sat Apr 25 16:49:47 2020
//	replace(file_name.begin(), file_name.end(), ':', '-');
//	replace(file_name.begin(), file_name.end(), '\0', ' ');
//	replace(file_name.begin(), file_name.end(), '\n', ' ');
//	meshio::SaveObj((BIN_DATA_PATH + file_name + ".obj").c_str(), X, facets);
//
//	cout << "Synthesize finished!" << endl;
//}
//
//void Reshaper::RFEMapping(Eigen::MatrixXd input_measure, Eigen::SparseMatrix<double> A, Eigen::Matrix3Xi &facets)
//{
//	//调用python脚本，预测尺寸信息
//	//py::scoped_interpreter python;
//	py::module py_test = py::module::import("reshaper");
//	py::object result = py_test.attr("mapping_rfemat")(input_measure);
//
//	//Eigen::MatrixXd output_measure = result.cast<Eigen::MatrixXd>();
//	Eigen::MatrixXd deform = result.cast<Eigen::MatrixXd>();
//	cout << deform.topRows(9) << endl;
//	Synthesize(A, deform, facets);
//}
