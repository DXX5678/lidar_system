#ifndef _IMU_H
#define _IMU_H
#endif
#pragma once
#include <Eigen/Dense>
#include<vector>

using namespace Eigen;
using namespace std;
namespace IMUissue
{
	//保留小数位操作
	double precision(double x, int y = 3);

	//读imu数据文件
	vector < Matrix<double, 7, 2000>>  Read(string name, int& total, int& rest);

	//imu数据插值
	void imu_timenew(vector < Matrix<double, 7, 2000>>& data, int total, int rest);
}