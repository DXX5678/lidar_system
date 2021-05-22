#include<iostream>
#include<fstream>
#include<iomanip>
#include "RSprocess.h"

using namespace std;
using namespace Eigen;
namespace Lidarissue {

	/*读lidar初始数据包，输入文件名，返回初始数据格式为元素为字符串的容器的列表*/
	list<vector<string>> lidar_reading(string filename)
	{
		vector<string> packet;
		list<vector<string>> Packets;
		int i;
		int sumtotal = 0;
		fstream fileName(filename, ios::in);
		if (!fileName.is_open())
		{
			throw "lidar文件打开失败！";
		}
		string Adata;
		getline(fileName, Adata, '\'');
		getline(fileName, Adata, '\'');
		while (!fileName.eof())
		{
			for (i = 0; i < 60 && !fileName.eof(); i++)
			{
				packet.push_back(Adata);
				sumtotal++;
				getline(fileName, Adata, '\'');
				getline(fileName, Adata, '\'');
			}
			i = 0;
			Packets.push_back(packet);
			packet.clear();
		}
		fileName.close();
		return Packets;
	}

	/*RS型雷达管理者类初始化，输入雷达类型和工作模式*/
	RSManager::RSManager(int dual_mode1, int lidar_type1)
	{
		dual_mode = dual_mode1;
		lidar_type = lidar_type1;
		try {
			timing_offsets = calc_timing_offsets();
		}
		catch (string mes)
		{
			throw mes;
		}
		frame_nr = 0;
		if (lidar_type == 16)
		{
			MatrixXd omegat(1, 16);
			omegat << -14.976, -12.997, -10.9859, -9.0065, -7.0016, -4.99, -3.0053, -0.9883, 15.0428, 13.0038, 11.0101, 8.9855, 6.9734, 4.99, 2.9624, 0.9918;
			omega = omegat;
			RS1 = RS(timing_offsets, dual_mode, lidar_type, omega);
		}
		else if (lidar_type == 32)
		{
			MatrixXd omegat(1, 32);
			omegat << -10.281, -6.424, 2.333, 3.297, 4.667, 6.947, 10.333, 14.967, 0.333, 0.0, -0.333, -0.667, 1.667, 1.333, 1.0, 0.667, -24.971, -14.638, -7.91, -5.407, -3.685, -4.018, -4.369, -4.685, -2.351, -2.685, -3.0, -3.333, -1.018, -1.351, -1.667, -2.0;
			omega = omegat;
			MatrixXd sigmt(1, 32);
			sigmt << 7.918, 8.009, 7.972, -7.696, 8.093, -7.807, 8.185, -8.008000000000001, -8.193, -2.925, 2.459, 7.778, -8.314, -2.98, 2.34, 7.692, -8.282, -8.051, -8.047, -8.136, -8.358, -3.091, 2.266, 7.64, -8.219, -2.944, 2.41, 7.824, -8.114, -2.884, 2.466, 7.793;
			sigm = sigmt;
			RS1 = RS(timing_offsets, dual_mode, lidar_type, omega, sigm);
		}
		else
		{
			throw "雷达型号输入错误，请选择RS16或RS32！";
		}
	}

	/*RS型雷达管理者的雷达处理函数，输入为读函数返回的初始数据，返回解析数据格式为元素为矩阵的列表*/
	void RSManager::lidar_manage(list<vector<string>> datalist, list<Matrix<double, 6, Dynamic >>& lidar_final)
	{
		Data DATAX;
		int sum = 0;
		long long int kum = 0;
		int col_num = 1;
		Matrix<double, 6, Dynamic>lidar_temp;
		lidar_temp.resize(6, 1);
		list<vector<string>>::iterator id = datalist.begin();
		DATAX = RS1.process_lidardata_frame((*id)[0]);
		double time = DATAX.timestamps[0];
		for (; id != datalist.end(); id++)
		{
			for (int i = 0; i < (*id).size(); i++)
			{
				DATAX = RS1.process_lidardata_frame((*id)[i]);
				for (int j = 0; j < 384; j++)
				{
					if ((DATAX.azimuth[j] < 90 || DATAX.azimuth[j]>270) && (DATAX.distances[j] < 200) && (DATAX.distances[j] > 15))
					{
						if ((time == DATAX.timestamps[j]))
						{
							lidar_temp(0, col_num - 1) = DATAX.timestamps[j];
							lidar_temp(1, col_num - 1) = DATAX.X[j];
							lidar_temp(2, col_num - 1) = DATAX.Y[j];
							lidar_temp(3, col_num - 1) = DATAX.Z[j];
							lidar_temp(4, col_num - 1) = DATAX.intensities[j];
							lidar_temp(5, col_num - 1) = DATAX.wavenumber[j];
							++col_num;
							time = DATAX.timestamps[j];
							lidar_temp.conservativeResize(6, col_num);
						}
						else
						{
							time = DATAX.timestamps[j];
							lidar_temp.conservativeResize(6, col_num - 1);
							lidar_final.push_back(lidar_temp);
							lidar_temp.resize(6, 1);
							col_num = 1;
							j--;
						}
					}

				}

			}
			(*id).clear();
			(*id).shrink_to_fit();
		}
		lidar_temp.conservativeResize(6, col_num - 1);
		lidar_final.push_back(lidar_temp);
		lidar_temp.resize(6, 1);
	}

	/*获得对应雷达类型的时间偏移量*/
	MatrixXd RSManager::calc_timing_offsets()
	{
		MatrixXd timing_offsets(32, 12);
		double full_firing_cycle;
		double single_firing;
		if (lidar_type == 16)
		{
			full_firing_cycle = 55.5;
			single_firing = 2.8;
			int dataBlockIndex;
			int dataPointIndex;
			for (int x = 0; x < 12; x++)
			{
				for (int y = 0; y < 32; y++)
				{
					if (dual_mode == 0)
						dataBlockIndex = (x - (x % 2)) + int((y / (16)));
					else
						dataBlockIndex = (x * 2) + int((y / (16)));
					dataPointIndex = y % 16;
					timing_offsets(y, x) = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
				}
			}
			return timing_offsets.transpose();
		}
		else if (lidar_type == 32)
		{
			full_firing_cycle = 55.52;
			single_firing = 1.44;
			for (int x = 0; x < 12; x++)
			{
				for (int y = 0; y < 32; y++)
				{
					if (dual_mode == 0)
						timing_offsets(y, x) = full_firing_cycle * floor(x / 2) + single_firing * 2 * (y % 16) + single_firing * floor((y + 1) / 16);
					else
						timing_offsets(y, x) = full_firing_cycle * x + single_firing * 2 * (y % 16) + single_firing * floor((y + 1) / 16);
				}
			}
			return timing_offsets.transpose();
		}
		else
		{
			throw "雷达型号输入错误，请选择RS16或RS32！";
		}
	}

	/*RS型雷达管理者类析构函数*/
	RSManager::~RSManager()
	{

	}
};



