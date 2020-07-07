#include "utils.h"

/*!
*@brief  将点集写入文件
*@param[out]
*@param[in]  const std::string & filename
*@param[in]  const std::vector<pmp::vec3> & points
*@return     int
*/
//int  SavePointToFile(const std::string &filename, const std::vector<pmp::vec3>& points)
//{
//	std::ofstream os(filename.c_str());
//	if (!os)
//		return 0;
//	for (auto& p : points)
//	{
//		os << p[0] << " " << p[1] << "\n";
//	}
//	os.close();
//	return 1;
//}

/*!
*@brief  读取文件夹下所有文件
*@param[out]
*@param[in]  const std::string & cate_dir  文件夹路径
*@return     std::vector<std::string>  文件夹下所有文件
*/


std::vector<std::string> GetFiles(const std::string & cate_dir)
{
	std::vector<std::string> files;

	_finddata_t file;
	intptr_t lf;
	// the type should be intptr_t in x64 machine
	// but it will be fine using long in x86

	if ((lf = _findfirst(cate_dir.c_str(), &file)) == -1) {
		std::cout << cate_dir << " not found!!!" << std::endl;
	}
	else {
		while (_findnext(lf, &file) == 0) {
			// cout<<file.name<<endl;
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			files.push_back(file.name);
		}
	}
	_findclose(lf);

	sort(files.begin(), files.end());
	return files;
}