#pragma once

#include <vector>
#include <time.h>
#include <string>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>

namespace  binaryio {
	template<class Vector, class IS>
	void ReadVectorBinary(IS &in, Vector& vec)
	{
		typename Vector::Index size = 0;
		in.read((char*)(&size), sizeof(typename Vector::Index));
		vec.resize(size);
		in.read((char *)vec.data(), size * sizeof(typename Vector::Scalar));
	}

	template<class Matrix, class IS>
	void ReadMatrixBinary(IS &in, Matrix& matrix)
	{
		typename Matrix::Index rows = 0, cols = 0;
		in.read((char*)(&rows), sizeof(typename Matrix::Index));
		in.read((char*)(&cols), sizeof(typename Matrix::Index));
		matrix.resize(rows, cols);
		in.read((char*)matrix.data(), rows*cols * sizeof(typename Matrix::Scalar));
	}

	template<class Vector, class OS>
	void WriteVectorBinary(OS &out, const Vector& vec)
	{
		typename Vector::Index size = vec.size();
		out.write((const char*)(&size), sizeof(typename Vector::Index));
		out.write((const char*)vec.data(), size * sizeof(typename Vector::Scalar));
	}

	template<class Matrix, class OS>
	void WriteMatrixBinary(OS &out, const Matrix& matrix)
	{
		typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
		out.write((const char*)(&rows), sizeof(typename Matrix::Index));
		out.write((const char*)(&cols), sizeof(typename Matrix::Index));
		out.write((const char*)matrix.data(), rows*cols * sizeof(typename Matrix::Scalar));
	}

	template<class Matrix>
	void ReadMatrixBinaryFromFile(const char *filename, Matrix &matrix)
	{
		std::ifstream is(filename, std::ios::binary);
		if (is)
		{
			ReadMatrixBinary(is, matrix);
		}
		is.close();
	}

	template<class Matrix>
	void WriteMatrixBinaryToFile(const char *filename, const Matrix &matrix)
	{
		std::ofstream out(filename, std::ios::binary);
		if (out)
		{
			WriteMatrixBinary(out, matrix);
		}
		out.close();
	}

	template<class Matrix>
	void ReadMatrixBinaryFromFileRowbyRow(const char *filename, Matrix &matrix)
	{
		std::ifstream is(filename, std::ios::binary);
		if (is) {
			size_t rows;
			is.read((char*)(&rows), sizeof(size_t));
			Eigen::VectorXd data;
			ReadVectorBinary(is, data);
			matrix.resize(data.size(), rows);
			matrix.col(0) = data;
			for (size_t i = 1; i < rows; ++i)
			{
				ReadVectorBinary(is, data);
				matrix.col(i) = data;
			}
		}
		is.close();
	}

	template<class Matrix>
	void WriteMatrixBinaryToFileRowbyRow(const char *filename, const Matrix &matrix)
	{
		std::ofstream out(filename, std::ios::binary);
		if (out)
		{
			const size_t rows = matrix.cols();
			out.write((const char*)(&rows), sizeof(size_t));
			for (size_t i = 0; i < rows; ++i)
			{
				WriteVectorBinary(out, matrix.col(i));
			}
		}
		out.close();
	}

	// @brief convert the config file (measurement) from ascii to binary
	// @param filein:input file name
	// @param fileout:output file name
	// @return 0/1
	template<class STRING>
	int TransFileAscii2Binary(const STRING &filein, const STRING &fileout)
	{
		std::ifstream is(filein, std::ios::binary);
		if (!is)
			return 0;
		while (!is.eof())
		{
			std::string line;
			std::getline(is, line);
			if (line.empty() || 13 == line[0])
				continue;
			std::istringstream instream(line);
			std::string word;
			instream >> word;
			if (word == "SIZE")
			{
				size_t num_bodies, num_features;
				instream >> num_bodies >> num_features;
				std::ofstream out(fileout, std::ios::out | std::ios::binary | std::ios::trunc);
				out.write((const char*)(&num_bodies), sizeof(size_t));
				Eigen::VectorXd data(num_features);
				for (size_t i = 0; i < num_bodies; ++i)
				{
					std::getline(is, line);
					std::istringstream instream(line);
					for (size_t j = 0; j < num_features; ++j)
					{
						instream >> data[j];
					}
					binaryio::WriteVectorBinary(out, data);
				}
				out.close();
				break;
			}
		}
		return 1;
	}

	// @brief convert the config file from binary to ascii
	// @param filein:input file name
	// @param fileout:output file name
	// @param num:number of bodies to be measured
	// @return 0/1
	template<class STRING>
	int TransFileBinary2Ascii(const STRING &filein, const STRING &fileout, size_t num)
	{
		std::ifstream in(filein.c_str(), std::ios::in | std::ios::binary);
		if (!in)
			return 0;
		size_t rows;
		std::ofstream out(fileout);
		out << "SIZE " << num;
		in.read((char*)(&rows), sizeof(size_t));
		Eigen::VectorXd row_data;
		binaryio::ReadVectorBinary(in, row_data);
		out << " " << row_data.size() << std::endl;
		out << row_data.transpose() << std::endl;
		for (size_t i = 1; i < num; ++i)
		{
			binaryio::ReadVectorBinary(in, row_data);
			out << row_data.transpose() << std::endl;
		}
		in.close();
		out.close();
		return 1;
	}

	// @brief convert the file saved with vectors row by row to matrix
	// @param filename:input file name
	// @param matrix:output matrix read from file
	// @return void
	template<class Matrix>
	void TransDataRowbyRow2Mat(const char *filename, Matrix &matrix)
	{
		std::ifstream is(filename);
		if (is)
		{
			size_t rows;
			Eigen::VectorXd row_data;
			is.read((char*)(&rows), sizeof(size_t));
			binaryio::ReadVectorBinary(is, row_data);
			matrix.resize(row_data.size(), rows);
			matrix.col(0) = row_data;
			for (size_t i = 1; i < rows; ++i)
			{
				binaryio::ReadVectorBinary(is, row_data);
				matrix.col(i) = row_data;
			}
		}
		is.close();
	}
}
