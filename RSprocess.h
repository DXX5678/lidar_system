#ifndef _lidar_H
#define _lidar_H
#endif
#pragma once
#include <vector>
#include <string>
#include<Eigen/Dense>
#include <list>
#include "RSanalyze.cpp"

using namespace std;
using namespace Eigen;
namespace Lidarissue {
	//读取lidar文件
	list<vector<string>> lidar_reading(string filename);

	//RS型雷达管理者类
	class RSManager
	{
	private:
		int dual_mode;
		int lidar_type;
		MatrixXd timing_offsets;
		int frame_nr;
		MatrixXd omega;
		MatrixXd sigm;
		RS RS1;
	public:
		RSManager(int dual_mode1 = 0, int lidar_type1 = 0);
		void lidar_manage(list<vector<string>> datalist, list<Matrix<double, 7, Dynamic >>& lidar_final);
		MatrixXd calc_timing_offsets();
		~RSManager();
	};
}
