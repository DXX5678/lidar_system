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
	extern int total;
	extern int rest;

	//保留小数位操作
	double precision(double x, int y = 3);

	//读imu数据文件
	vector < Matrix<double, 7, 2000>>  Read(string name);

	//读imu数据文件版本2
	vector < Matrix<double, 7, 2000>>  Read_2(string name);

	//imu数据插值
	void imu_timenew(vector < Matrix<double, 7, 2000>>& data);

	//角度转换
	double transition(double x, double y, double z, int h = 3);

	//读imu数据txt版本
	vector < Matrix<double, 7, 2000>>  Read_txt(string name);
}