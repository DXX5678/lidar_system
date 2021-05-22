#include <string.h>
#include <string>
#include <cassert>
#include <Eigen/Core>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <atltime.h>
#include <cmath>
#include <corecrt_math_defines.h>
#include <iostream>
#include<iomanip>
using namespace std;

/*lidar_RS解析后数据包*/
struct Data
{
	Eigen::RowVectorXd wavenumber;
	Eigen::RowVectorXd X;
	Eigen::RowVectorXd Y;
	Eigen::RowVectorXd Z;
	Eigen::RowVectorXd intensities;
	Eigen::RowVectorXd azimuth;
	Eigen::RowVectorXd timestamps;
	Eigen::RowVectorXd distances;
};

/*Lidar类*/
class Lidar
{
public:
	struct Data process_lidardata_frame(string data) {};
};

/*RS32/16类*/
class RS :public Lidar
{
public:
	double RAD2D = 180 / M_PI;
	double FACTOR_CM2M_16 = 0.005;
	double FACTOR_CM2M_32 = 0.005;
	Eigen::MatrixXd timing_offsets;
	int count_lasers;
	int dual_modle;
	Eigen::MatrixXd omega;
	Eigen::MatrixXd sigm;
	/*构造函数：完成原始参数初始化*/
	RS()
	{

	}
	RS(Eigen::Matrix<double, 12, 32> timing_offsets, int dual_mode, int lidar_type, Eigen::MatrixXd omega, Eigen::MatrixXd sigm = Eigen::MatrixXd::Ones(1, 16))
	{
		this->timing_offsets = timing_offsets;
		this->count_lasers = lidar_type;
		this->omega = omega;
		this->sigm = sigm;
		this->dual_modle = dual_mode;
	}
	/*保留三位小数*/
	Eigen::Matrix<double, 12, 32, Eigen::RowMajor> precision(Eigen::Matrix<double, 12, 32, Eigen::RowMajor> x, int y = 3)
	{
		double temp = pow(0.1, y + 1);
		x = x.array() + (temp * 5);
		int temp2 = pow(10, y);
		Eigen::Matrix<int, 12, 32> z = (x * temp2).cast<int>();
		x = z.cast<double>() / temp2;
		return x;
	}

	/*一维向量转矩阵*/
	void ConvertToEigenVector(Eigen::Matrix<double, 1, 12>& result, std::vector<double > data)
	{
		result.row(0) = Eigen::VectorXd::Map(&data[0], data.size());
	}

	/*二维向量转矩阵*/
	Eigen::MatrixXd ConvertToEigenMatrix(std::vector<std::vector<double>>data)
	{
		Eigen::MatrixXd eMatrix(data.size(), data[0].size());
		for (unsigned int i = 0; i < data.size(); ++i)
		{
			eMatrix.row(i) = Eigen::VectorXd::Map(&data[i][0], data[i].size());
			data[i].clear();
			data[i].shrink_to_fit();
		}
		return eMatrix;
	}

	/*分割字符串(取单个字节数据)*/
	void split_by_length(string* result, string str, int length)
	{
		int num = int(str.length()) / length;
		for (int i = 0; i < num; i++)
		{
			result[i] = str.substr(static_cast<std::basic_string<char, std::char_traits<char>, std::allocator<char>>::size_type>(i) * length, length);
		}
	}

	/*时间戳转换 s 秒*/
	double timeconvert(int week, int hour, double minute, double second, double ms, double us)
	{
		double timestamp = week * 86400 + hour * 3600 + minute * 60 + second + ms / 1000 + us / 1000000 + 18;
		return timestamp;
	}

	/*得到原始时间戳*/
	double get_timestamp(string timesp)
	{
		int year = strtol(timesp.substr(0, 2).data(), NULL, 16);
		year += 2000;
		int month = strtol(timesp.substr(2, 2).data(), NULL, 16);
		int day = strtol(timesp.substr(4, 2).data(), NULL, 16);
		int hour = strtol(timesp.substr(6, 2).data(), NULL, 16);
		double minute = strtol(timesp.substr(8, 2).data(), NULL, 16);
		double second = strtol(timesp.substr(10, 2).data(), NULL, 16);
		double ms = strtol(timesp.substr(12, 4).data(), NULL, 16);
		double us = strtol(timesp.substr(16, 4).data(), NULL, 16);
		CTime time(year, month, day, 0, 0, 0);
		int week = time.GetDayOfWeek() - 1;
		double timestamp = timeconvert(week, hour, minute, second, ms, us);
		return timestamp;
	}

	/*得到最终时间戳矩阵*/
	Eigen::Matrix<double, 12, 32> get_timestamps(Eigen::Matrix<double, 12, 32> timing_offsets, double timestamp)
	{
		Eigen::Matrix<int, 12, 32>* mid = new (Eigen::Matrix<int, 12, 32>);
		Eigen::Matrix<double, 12, 32> timestamps = (timing_offsets / 1000000).array() + timestamp;
		*mid = ((timestamps.array() + 0.0005) * 1000).cast<int>();
		timestamps = ((*mid).cast<double>()) / 1000;
		delete(mid);
		return timestamps;
	}

	/*读lidar原始数据得到距离，反射率，水平角*/
	void read_firing_data(Eigen::Matrix<string, 1, 100> data, std::vector<double>& distances, std::vector<double>& intensities, std::vector<double>& azimuth_per_block)
	{
		if (count_lasers == 16)
		{
			assert(data[0] + data[1] == "ffee");
			azimuth_per_block.push_back(strtol((data[2] + data[3]).data(), NULL, 16) / static_cast<double>(100));
			for (int i = 1; i <= 32; i++)
			{
				distances.push_back(strtol((data[3 * static_cast<Eigen::Index>(i) + 1] + data[3 * static_cast<Eigen::Index>(i) + 2]).data(), NULL, 16) * FACTOR_CM2M_16);
				intensities.push_back(strtol((data[3 * static_cast<Eigen::Index>(i) + 3]).data(), NULL, 16));
			}
		}
		else
		{
			assert(data[0] + data[1] == "ffee");
			azimuth_per_block.push_back(strtol((data[2] + data[3]).data(), NULL, 16) / static_cast<double>(100));
			for (int i = 1; i <= 32; i++)
			{
				distances.push_back(strtol((data[3 * static_cast<Eigen::Index>(i) + 1] + data[3 * static_cast<Eigen::Index>(i) + 2]).data(), NULL, 16) * FACTOR_CM2M_32);
				intensities.push_back(strtol((data[3 * static_cast<Eigen::Index>(i) + 3]).data(), NULL, 16));
			}
		}
	}

	/*得到最终水平角矩阵*/
	Eigen::Matrix<double, 12, 32> calc_precise_azimuth(Eigen::Matrix<double, 1, 12>azimuth_per_matrix)
	{
		Eigen::Matrix<double, 12, 32>azimuth;
		Eigen::VectorXd* vec = new Eigen::VectorXd();
		(*vec).setLinSpaced(32, 0, 31);
		Eigen::Matrix<double, 1, 12>* temp = new Eigen::Matrix<double, 1, 12>();
		*temp = azimuth_per_matrix;
		for (int i = 0; i < azimuth_per_matrix.size(); i++)
		{
			double azimuth_gap;
			try {
				if (i == 11)
				{
					if ((*temp)(0, i) < (*temp)(0, static_cast<Eigen::Index>(i) - 1))
						(*temp)(0, i) += 360;
					azimuth_gap = (*temp)(0, i) - (*temp)(0, static_cast<Eigen::Index>(i) - 1);
				}
				else
				{
					if ((*temp)(0, static_cast<Eigen::Index>(i) + 1) < (*temp)(0, i))
						(*temp)(0, static_cast<Eigen::Index>(i) + 1) += 360;
					azimuth_gap = (*temp)(0, static_cast<Eigen::Index>(i) + 1) - (*temp)(0, i);
				}
			}
			catch (exception) {
				if ((*temp)(0, i) < (*temp)(0, static_cast<Eigen::Index>(i) - 1))
					(*temp)(0, i) += 360;
				azimuth_gap = (*temp)(0, i) - (*temp)(0, static_cast<Eigen::Index>(i) - 1);
			}
			double factor = azimuth_gap / 32;
			azimuth.row(i) = (((*vec) * factor).array() + (*temp)(0, i)).transpose();
		}
		delete(vec);
		delete(temp);
		return azimuth;
	}

	/*距离矩阵转XYZ坐标矩阵*/
	void calc_cart_coord(Eigen::Matrix<double, 12, 32>distances, Eigen::Matrix<double, 12, 32>azimuth, Eigen::Matrix<double, 12, 32, Eigen::RowMajor>& X, Eigen::Matrix<double, 12, 32, Eigen::RowMajor>& Y, Eigen::Matrix<double, 12, 32, Eigen::RowMajor>& Z)
	{
		Eigen::MatrixXd longitudes(1, 32);
		Eigen::MatrixXd latitudes(12, 32);
		if (count_lasers == 16)
		{
			longitudes << omega * M_PI / 180, omega* M_PI / 180;
			latitudes = azimuth * M_PI / 180;
		}
		else
		{
			longitudes = omega * M_PI / 180;
			Eigen::Map<Eigen::VectorXd> vec(sigm.data(), sigm.size());
			latitudes = (azimuth.rowwise() + vec.transpose()) * M_PI / 180;
		}
		Eigen::MatrixXd hypotenuses = distances.array().rowwise() * cos(Eigen::Map<Eigen::VectorXd>(longitudes.data(), longitudes.size()).array()).transpose().array();
		X = precision(hypotenuses.array() * sin(latitudes.array()).array());
		Y = precision(hypotenuses.array() * cos(latitudes.array()).array());
		Z = precision(distances.array().rowwise() * sin(Eigen::Map<Eigen::VectorXd>(longitudes.data(), longitudes.size()).array()).transpose().array());
	}

	/*处理lidar原始数据包得到最终lidar数据包Data*/
	struct Data process_lidardata_frame(string data)
	{
		if (dual_modle == 1)
		{
			try {
				if (data.length() == 2496)
				{
					string header = data.substr(0, 84);
					assert(header.substr(0, 4) == "55aa");
					assert(data.substr(2492, 4) == "00ff");
					double timestamp = get_timestamp(header.substr(40, 20));
					int num = data.substr(84, 2400).length();
					string* result = new string[num];
					split_by_length(result, data.substr(84, 2400), 2);
					Eigen::Matrix<string, 12, 100, Eigen::RowMajor> datamatrix = Eigen::Map<Eigen::Matrix<string, 12, 100, Eigen::RowMajor>>(result, 12, 100);
					delete[]result;
					std::vector<std::vector<double>> distances(12);
					std::vector<std::vector<double>> intensities(12);
					std::vector<double> azimuth_per_block;
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* wave = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					for (int i = 0; i < 12; i++)
					{
						if (i % 2 == 0)
						{
							wave->row(i).setOnes();
						}
						else
						{
							wave->row(i) = (wave->row(i).setOnes()).array() + 1;
						}
						read_firing_data((datamatrix).row(i), distances[i], intensities[i], azimuth_per_block);
					}
					Eigen::Matrix<double, 1, 12>* azimuth_per_matrix = (Eigen::Matrix<double, 1, 12>*)malloc(sizeof(Eigen::Matrix<double, 1, 12>));
					ConvertToEigenVector(*azimuth_per_matrix, azimuth_per_block);
					azimuth_per_block.clear();
					azimuth_per_block.shrink_to_fit();
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* azimuth_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*azimuth_matrix = calc_precise_azimuth(*azimuth_per_matrix);
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* distances_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*distances_matrix = ConvertToEigenMatrix(distances);
					distances.clear();
					distances.shrink_to_fit();
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* intensities_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*intensities_matrix = ConvertToEigenMatrix(intensities);
					intensities.clear();
					intensities.shrink_to_fit();
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* Xm = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* Ym = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* Zm = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					calc_cart_coord(*distances_matrix, *azimuth_matrix, *Xm, *Ym, *Zm);
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* timestamps_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*timestamps_matrix = get_timestamps(timing_offsets, timestamp);
					struct Data d;
					d.wavenumber = Eigen::Map<Eigen::RowVectorXd>((*wave).data(), (*wave).size());
					d.X = Eigen::Map<Eigen::RowVectorXd>((*Xm).data(), (*Xm).size());
					d.Y = Eigen::Map<Eigen::RowVectorXd>((*Ym).data(), (*Ym).size());
					d.Z = Eigen::Map<Eigen::RowVectorXd>((*Zm).data(), (*Zm).size());
					d.intensities = Eigen::Map<Eigen::RowVectorXd>((*intensities_matrix).data(), (*intensities_matrix).size());
					d.azimuth = Eigen::Map<Eigen::RowVectorXd>((*azimuth_matrix).data(), (*azimuth_matrix).size());
					d.timestamps = Eigen::Map<Eigen::RowVectorXd>((*timestamps_matrix).data(), (*timestamps_matrix).size());
					d.distances = Eigen::Map<Eigen::RowVectorXd>((*distances_matrix).data(), (*distances_matrix).size());
					free(wave);
					free(azimuth_per_matrix);
					free(azimuth_matrix);
					free(distances_matrix);
					free(intensities_matrix);
					free(Xm);
					free(Ym);
					free(Zm);
					free(timestamps_matrix);
					return d;
				}
				else
				{
					struct Data d;
					return d;
				}
			}
			catch (exception) {
				struct Data d;
				return d;
			}
		}
		else if (dual_modle == 0)
		{
			try {
				if (data.length() == 2496)
				{
					string header = data.substr(0, 84);
					assert(header.substr(0, 4) == "55aa");
					assert(data.substr(2492, 4) == "00ff");
					double timestamp = get_timestamp(header.substr(40, 20));
					int num = data.substr(84, 2400).length();
					string* result = new string[num];
					split_by_length(result, data.substr(84, 2400), 2);
					Eigen::Matrix<string, 12, 100, Eigen::RowMajor> datamatrix = Eigen::Map<Eigen::Matrix<string, 12, 100, Eigen::RowMajor>>(result, 12, 100);
					delete[]result;
					std::vector<std::vector<double>> distances(12);
					std::vector<std::vector<double>> intensities(12);
					std::vector<double> azimuth_per_block;
					for (int i = 0; i < 12; i++)
					{
						read_firing_data((datamatrix).row(i), distances[i], intensities[i], azimuth_per_block);
					}
					Eigen::Matrix<double, 1, 12>* azimuth_per_matrix = (Eigen::Matrix<double, 1, 12>*)malloc(sizeof(Eigen::Matrix<double, 1, 12>));
					ConvertToEigenVector(*azimuth_per_matrix, azimuth_per_block);
					azimuth_per_block.clear();
					azimuth_per_block.shrink_to_fit();
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* azimuth_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*azimuth_matrix = calc_precise_azimuth(*azimuth_per_matrix);
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* distances_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*distances_matrix = ConvertToEigenMatrix(distances);
					distances.clear();
					distances.shrink_to_fit();
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* intensities_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*intensities_matrix = ConvertToEigenMatrix(intensities);
					intensities.clear();
					intensities.shrink_to_fit();
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* Xm = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* Ym = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* Zm = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					calc_cart_coord(*distances_matrix, *azimuth_matrix, *Xm, *Ym, *Zm);
					Eigen::Matrix<double, 12, 32, Eigen::RowMajor>* timestamps_matrix = (Eigen::Matrix<double, 12, 32, Eigen::RowMajor>*)malloc(sizeof(Eigen::Matrix<double, 12, 32, Eigen::RowMajor>));
					*timestamps_matrix = get_timestamps(timing_offsets, timestamp);
					struct Data d;
					d.X = Eigen::Map<Eigen::RowVectorXd>((*Xm).data(), (*Xm).size());
					d.Y = Eigen::Map<Eigen::RowVectorXd>((*Ym).data(), (*Ym).size());
					d.Z = Eigen::Map<Eigen::RowVectorXd>((*Zm).data(), (*Zm).size());
					d.intensities = Eigen::Map<Eigen::RowVectorXd>((*intensities_matrix).data(), (*intensities_matrix).size());
					d.azimuth = Eigen::Map<Eigen::RowVectorXd>((*azimuth_matrix).data(), (*azimuth_matrix).size());
					d.timestamps = Eigen::Map<Eigen::RowVectorXd>((*timestamps_matrix).data(), (*timestamps_matrix).size());
					d.distances = Eigen::Map<Eigen::RowVectorXd>((*distances_matrix).data(), (*distances_matrix).size());
					d.wavenumber.setOnes(384);
					free(azimuth_per_matrix);
					free(azimuth_matrix);
					free(distances_matrix);
					free(intensities_matrix);
					free(Xm);
					free(Ym);
					free(Zm);
					free(timestamps_matrix);
					return d;
				}
				else
				{
					struct Data d;
					return d;
				}
			}
			catch (exception) {
				struct Data d;
				return d;
			}
		}
		else
		{
			throw "雷达工作类型输入错误，请输入0或者1对应单回波或者双回波！";
		}
	};
};
