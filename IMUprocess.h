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

	//��imu�����ļ��汾2
	vector < Matrix<double, 7, 2000>>  Read_2(string name);

	//imu���ݲ�ֵ
	void imu_timenew(vector < Matrix<double, 7, 2000>>& data);

	//�Ƕ�ת��
	double transition(double x, double y, double z, int h = 3);

	//��imu����txt�汾
	vector < Matrix<double, 7, 2000>>  Read_txt(string name);
}