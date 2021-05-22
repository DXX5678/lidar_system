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
	extern Matrix< double, 6, 6> RotationMatrix; //旋转矩阵
	extern Matrix< double, 6, 6> anzhiMatrix; //安置矩阵

	//获取旋转矩阵 
	void GetRotationMatrix(double, double, double);

	//得到安置矩阵
	void Get_anzhi_RotationMatrix(double a = 0, double b = 0, double c = 0);

	//角度转弧度
	double radian(double);

	/*对imu数据进行去拐点操作*/
	vector <MatrixXd> imudetach(vector < Matrix<double, 7, 2000>> data, int total, int rest);

	/*imu与lidar进行时间匹配*/
	void time_match(MatrixXd& imu, MatrixXd& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);

	/*将坐标转换为绝对坐标系下*/
	void Act(Matrix<double, 6, Dynamic >& result_spilt, double xx, double yy, double zz, double r, double p, double h, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);
	
	//矩阵平移
	void Parallel(Matrix<double, 6, Dynamic >& result_spilt, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z);
}