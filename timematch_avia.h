#ifndef _TIMEA_H
#define _TIMEA_H
#endif
#pragma once
#include <Eigen/Dense>
#include<vector>
#include<list>
using namespace Eigen;
using namespace std;
namespace TIMEA
{
	extern Matrix< double, 6, 6> RotationMatrix; //��ת����
	extern Matrix< double, 6, 6> anzhiMatrix; //���þ���
	extern Matrix<double, 6, 6> Rw; //Rw����

	//��ȡ��ת���� 
	void GetRotationMatrix(double, double, double);

	//�õ����þ���
	void AGet_anzhi_RotationMatrix(double a = 0, double b = 0, double c = 0);

	//�õ�Rw����
	void GetRwMatrix(double, double);

	//�Ƕ�ת����
	double radian(double);

	/*��imu���ݽ���ȥ�յ����*/
	vector <MatrixXd> Aimudetach(vector < Matrix<double, 7, 2000>> data, int total, int rest);

	/*��imu���ݽ��з��뺽������*/
	vector <MatrixXd> Aimudetach_2(vector < Matrix<double, 7, 2000>> data, int total, int rest);
	
	/*������ת��Ϊ��������ϵ��*/
	void Act(Matrix<double, 6, Dynamic >& result_spilt, double xx, double yy, double zz, double r, double p, double h, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	//����ƽ��
	void Parallel(Matrix<double, 6, Dynamic >& result_spilt, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	//��γ��ת��������
	vector<double>LLA2ECEF(double, double, double);

	//����ת���ɾ�γ��
	vector<double>ECEF2LLA(double, double, double);

	double rad2angle(double r);

	double angle2rad(double a);

	/*imu��lidar����ʱ��ƥ�� �б���ʽ*/
	void Atime_match_m_l(MatrixXd& imu, list<Matrix<double, 6, Dynamic>>& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	/*imu��lidar����ʱ��ƥ��ͬʱ����txt�ļ� �б���ʽ*/
	void Atime_match_m_l_txt(MatrixXd& imu, list<Matrix<double, 6, Dynamic>>& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z, int number = 1);
}