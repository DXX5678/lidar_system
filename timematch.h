#ifndef _TIMEM_H
#define _TIMEM_H
#endif
#pragma once
#include <Eigen/Dense>
#include<vector>
#include<list>
using namespace Eigen;
using namespace std;
namespace TIME
{
	extern Matrix< double, 6, 6> RotationMatrix; //��ת����
	extern Matrix< double, 6, 6> anzhiMatrix; //���þ���

	//��ȡ��ת���� 
	void GetRotationMatrix(double, double, double);

	//�õ����þ���
	void Get_anzhi_RotationMatrix(double a = 0, double b = 0, double c = 0);

	//�Ƕ�ת����
	double radian(double);

	/*��imu���ݽ���ȥ�յ����*/
	vector <MatrixXd> imudetach(vector < Matrix<double, 7, 2000>> data, int total, int rest);

	/*imu��lidar����ʱ��ƥ��*/
	void time_match(MatrixXd& imu, MatrixXd& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	/*������ת��Ϊ��������ϵ��*/
	void Act(Matrix<double, 6, Dynamic >& result_spilt, double xx, double yy, double zz, double r, double p, double h, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);
	
	//����ƽ��
	void Parallel(Matrix<double, 6, Dynamic >& result_spilt, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);
}