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
	extern Matrix< double, 6, 6> RotationMatrix; //旋转矩阵
	extern Matrix< double, 6, 6> anzhiMatrix; //安置矩阵
	extern Matrix<double, 6, 6> Rw; //Rw矩阵

	//获取旋转矩阵 
	void GetRotationMatrix(double, double, double);

	//得到安置矩阵
	void AGet_anzhi_RotationMatrix(double a = 0, double b = 0, double c = 0);

	//得到Rw矩阵
	void GetRwMatrix(double, double);

	//角度转弧度
	double radian(double);

	/*对imu数据进行去拐点操作*/
	vector <MatrixXd> Aimudetach(vector < Matrix<double, 7, 2000>> data, int total, int rest);

	/*对imu数据进行分离航带操作*/
	vector <MatrixXd> Aimudetach_2(vector < Matrix<double, 7, 2000>> data, int total, int rest);
	
	/*将坐标转换为绝对坐标系下*/
	void Act(Matrix<double, 6, Dynamic >& result_spilt, double xx, double yy, double zz, double r, double p, double h, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	//矩阵平移
	void Parallel(Matrix<double, 6, Dynamic >& result_spilt, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	//经纬度转换成坐标
	vector<double>LLA2ECEF(double, double, double);

	//坐标转换成经纬度
	vector<double>ECEF2LLA(double, double, double);

	double rad2angle(double r);

	double angle2rad(double a);

	/*imu与lidar进行时间匹配 列表形式*/
	void Atime_match_m_l(MatrixXd& imu, list<Matrix<double, 6, Dynamic>>& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	/*imu与lidar进行时间匹配同时生成txt文件 列表形式*/
	void Atime_match_m_l_txt(MatrixXd& imu, list<Matrix<double, 6, Dynamic>>& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z, int number = 1);
}