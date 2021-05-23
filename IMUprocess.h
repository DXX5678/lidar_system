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

	//����С��λ����
	double precision(double x, int y = 3);

	//��imu�����ļ�
	vector < Matrix<double, 7, 2000>>  Read(string name);

	//imu���ݲ�ֵ
	void imu_timenew(vector < Matrix<double, 7, 2000>>& data);
}