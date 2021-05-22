#include <iostream>
#include <fstream>
#include<windows.h>
#include<iomanip>
#include <algorithm>
#include <corecrt_math_defines.h>
#include <math.h>
#include "timematch.h"
#include <Windows.h>


using namespace Eigen;
using namespace std;

namespace TIME
{
	Matrix< double, 6, 6> RotationMatrix; //旋转矩阵
	Matrix< double, 6, 6> anzhiMatrix; //安置矩阵

	/*对imu数据进行去拐点操作*/
	vector <MatrixXd> imudetach(vector < Matrix<double, 7, 2000>> data, int total, int rest)
	{
		MatrixXd final;
		for (int i = 0; i < total - 1; i++)
		{
			final.conservativeResize(7, 2000 * (i + 1));
			final.block(0, i * 2000, 7, 2000) = data[i];
		}
		final.conservativeResize(7, 2000 * (total - 1) + rest);
		final.block(0, (total - 1) * 2000, 7, rest) = data[total - 1].block(0, 0, 7, rest);
		data.clear();
		data.shrink_to_fit();
		vector <MatrixXd> results;
		long long int xypoint = 0;
		long long int xpoint = 0;
		long long int ypoint = 0;
		vector < MatrixXd> xylanes;
		vector < MatrixXd> xlanes;
		vector < MatrixXd> ylanes;
		int width = 1;
		MatrixXd temp;
		bool find = false;
		for (int i = 0; i < final.cols() - 1; i++)
		{
			if (fabs(final(1, i) - final(1, i + 1)) < 0.001)
			{
				if (!find)
				{
					find = true;
				}
				temp.conservativeResize(7, width);
				temp.block(0, width - 1, 7, 1) = final.col(i);
				width++;
				xpoint++;
			}
			else
			{
				if (find)
				{
					xlanes.push_back(temp);
					width = 1;
					temp.resize(7, 1);
					find = false;
				}
			}
		}
		width = 1;
		find = false;
		temp.resize(7, 1);
		for (int i = 0; i < final.cols() - 1; i++)
		{
			if (fabs(final(2, i) - final(2, i + 1)) < 0.001)
			{
				if (!find)
				{
					find = true;
				}
				temp.conservativeResize(7, width);
				temp.block(0, width - 1, 7, 1) = final.col(i);
				width++;
				ypoint++;
			}
			else
			{
				if (find)
				{
					ylanes.push_back(temp);
					width = 1;
					temp.resize(7, 1);
					find = false;
				}
			}
		}
		width = 1;
		find = false;
		temp.resize(7, 1);
		for (int i = 0; i < final.cols() - 2; i++)
		{
			if (fabs((final(2, i + 1) - final(2, i)) / (final(1, i + 1) - final(1, i)) - (final(2, i + 2) - final(2, i + 1)) / (final(1, i + 2) - final(1, i + 1))) < 0.1)
			{
				if (!find)
				{
					find = true;
				}
				temp.conservativeResize(7, width);
				temp.block(0, width - 1, 7, 1) = final.col(i);
				width++;
				xypoint++;
			}
			else
			{
				if (find)
				{
					xylanes.push_back(temp);
					width = 1;
					temp.resize(7, 1);
					find = false;
				}
			}
		}
		long long int xy = max(xpoint, ypoint);
		vector < MatrixXd> lanes;
		if (xy > xypoint)
		{
			lanes = xpoint > ypoint ? xlanes : ylanes;
		}
		else
		{
			lanes = xylanes;
		}
		long long int count = 0;
		for (int j = 0; j < lanes.size(); j++)
		{
			count = max(count, lanes[j].cols());
		}
		for (int i = 0; i < lanes.size(); i++)
		{
			if (fabs(count - lanes[i].cols()) < count / 10)
			{
				results.push_back(lanes[i]);
			}
		}
		return results;
	}

	/*imu与lidar进行时间匹配*/
	void time_match(MatrixXd& imu, MatrixXd& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z)
	{

		long long int lidarcols = lidar.cols();
		long long int imucols = imu.cols();
		long long int i_imu = 0;
		long long int i_lidar = 0;//循环到第几个
		while (i_lidar <= lidarcols && i_imu <= imucols)
		{
			if (lidar(0, 0) < imu(0, 0))
			{
				lidar = lidar.block(0, 1, 6, lidarcols - 1);
				lidarcols--;
				continue;
			}
			if (lidar(0, 0) > imu(0, imucols - 1))
			{
				lidar = lidar.block(0, 1, 6, lidarcols - 1);
				lidarcols--;
				continue;
			}
			if (fabs(lidar(0, i_lidar) - imu(0, i_imu)) < 1e-3)
			{
				Matrix<double, 6, Dynamic>* temp = (Matrix<double, 6, Dynamic>*) (&(lidar.col(i_lidar)));
				Act((*temp), imu(1, i_imu), imu(2, i_imu), imu(3, i_imu), imu(4, i_imu), imu(5, i_imu), imu(6, i_imu), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
				i_lidar++;
			}
			else if (lidar(0, i_lidar) > imu(0, i_imu))
			{
				i_imu++;
				if (lidar(0, i_lidar) < imu(0, i_imu))
				{
					i_imu--;
					if (fabs(lidar(0, i_lidar) - imu(0, i_imu)) > 1e-3 && fabs(lidar(0, i_lidar) - imu(0, i_imu)) < 2 * 1e-3)
					{
						Matrix<double, 6, Dynamic>* temp = (Matrix<double, 6, Dynamic>*) (&(lidar.col(i_lidar)));
						Act((*temp), imu(1, i_imu), imu(2, i_imu), imu(3, i_imu), imu(4, i_imu), imu(5, i_imu), imu(6, i_imu), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
						i_imu++;
						i_lidar++;
					}
					else
					{
						MatrixXd problock = lidar.block(0, 0, 6, i_lidar);
						MatrixXd resblock = lidar.block(0, i_lidar + 1, 6, lidarcols - 1 - problock.cols());
						lidar.resize(6, lidarcols - 1);
						lidar.block(0, 0, 6, problock.cols()) = problock;
						lidar.block(0, i_lidar, 6, resblock.cols()) = resblock;
						lidarcols--;
					}
				}
			}
			else
			{
				i_imu--;
				if (lidar(0, i_lidar) > imu(0, i_imu))
				{
					i_imu++;
					if (fabs(lidar(0, i_lidar) - imu(0, i_imu)) > 1e-3 && fabs(lidar(0, i_lidar) - imu(0, i_imu)) < 2 * 1e-3)
					{
						Matrix<double, 6, Dynamic>* temp = (Matrix<double, 6, Dynamic>*) (&(lidar.col(i_lidar)));
						Act((*temp), imu(1, i_imu), imu(2, i_imu), imu(3, i_imu), imu(4, i_imu), imu(5, i_imu), imu(6, i_imu), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
						i_lidar++;
					}
					else
					{
						i_imu--;
						MatrixXd problock = lidar.block(0, 0, 6, i_lidar);
						MatrixXd resblock = lidar.block(0, i_lidar + 1, 6, lidarcols - 1 - problock.cols());
						lidar.resize(6, lidarcols - 1);
						lidar.block(0, 0, 6, problock.cols()) = problock;
						lidar.block(0, i_lidar, 6, resblock.cols()) = resblock;
						lidarcols--;
					}
				}
			}
		}
	}

	/*将坐标转换为绝对坐标系下*/
	void Act(Matrix<double, 6, Dynamic >& result_spilt, double xx, double yy, double zz, double r, double p, double h, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z)
	{

		//以下准备做安置
		result_spilt = anzhiMatrix * result_spilt;

		//以下准备做平移
		Parallel(result_spilt, CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);

		GetRotationMatrix(r, p, h);
		//以下准备做旋转
		result_spilt = RotationMatrix * result_spilt;

		int t = result_spilt.cols() - 1;
		Matrix<double, Dynamic, Dynamic>temp(1, t);
		temp = result_spilt.row(1);
		result_spilt.row(1) = result_spilt.row(2);
		result_spilt.row(2) = temp;
		result_spilt.row(3) = result_spilt.row(3) * -1;

		for (int i = 0; i < result_spilt.cols(); i++)
		{
			result_spilt(1, i) += yy;
			result_spilt(2, i) += xx;
			result_spilt(3, i) += zz;
		}
	}

	void GetRotationMatrix(double R, double P, double H)				//获取旋转矩阵 
	{
		R = radian(R);
		P = radian(P);
		H = radian(H);
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
			{
				if (i == j)RotationMatrix(i, j) = 1;
				else RotationMatrix(i, j) = 0;
			}
		RotationMatrix(1, 1) = cos(H) * cos(P);
		RotationMatrix(1, 2) = -sin(H) * cos(R) + cos(H) * sin(P) * sin(R);
		RotationMatrix(1, 3) = sin(H) * sin(R) + cos(H) * sin(P) * cos(R);
		RotationMatrix(2, 1) = sin(H) * cos(P);
		RotationMatrix(2, 2) = cos(H) * cos(R) + sin(H) * sin(P) * sin(R);
		RotationMatrix(2, 3) = sin(H) * sin(P) * cos(R) - cos(H) * sin(R);
		RotationMatrix(3, 1) = -sin(P);
		RotationMatrix(3, 2) = cos(P) * sin(R);
		RotationMatrix(3, 3) = cos(P) * cos(R);
	}

	double radian(double degree)									//角度转弧度
	{
		double flag = (degree < 0) ? -1.0 : 1.0;					//判断正负
		if (degree < 0)
		{
			degree = degree * (-1.0);
		}
		double angle = degree;
		double result = flag * (angle * M_PI) / 180;
		return result;
	}

	void Get_anzhi_RotationMatrix(double R, double P, double H)		//得到安置矩阵
	{
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				if (i == j)anzhiMatrix(i, j) = 1;
				else anzhiMatrix(i, j) = 0;
		anzhiMatrix(1, 2) = -radian(H);
		anzhiMatrix(1, 3) = radian(P);
		anzhiMatrix(2, 1) = radian(H);
		anzhiMatrix(2, 3) = -radian(R);
		anzhiMatrix(3, 1) = -radian(P);
		anzhiMatrix(3, 2) = radian(R);
	}

	void Parallel(Matrix<double, 6, Dynamic >& result_spilt, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z)     //矩阵平移
	{
		int t = result_spilt.cols() - 1;
		for (int i = 0; i < t; i++)
		{
			result_spilt(3, i) += CarrierToLidar_x;
			result_spilt(2, i) += CarrierToLidar_z;
			result_spilt(1, i) += CarrierToLidar_y;
		}
		Matrix<double, Dynamic, Dynamic>temp(1, t);
		temp = result_spilt.row(1);
		result_spilt.row(1) = result_spilt.row(3);
		result_spilt.row(3) = result_spilt.row(2);
		result_spilt.row(2) = temp;
	}

}







