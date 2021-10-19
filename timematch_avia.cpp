#include <iostream>
#include <fstream>
#include<windows.h>
#include<iomanip>
#include <algorithm>
#include <corecrt_math_defines.h>
#include <math.h>
#include "timematch_avia.h"
#include "IMUanalyze.h"
#include <Windows.h>


using namespace Eigen;
using namespace std;
using namespace conversion;

namespace TIMEA
{
	Matrix< double, 6, 6> RotationMatrix; //旋转矩阵
	Matrix< double, 6, 6> anzhiMatrix; //安置矩阵
	Matrix<double, 6, 6> Rw; //Rw矩阵

	/*对imu数据进行去拐点操作*/
	vector <MatrixXd> Aimudetach(vector < Matrix<double, 7, 2000>> data, int total, int rest)
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

		/*
		ofstream outfile("heading2.txt", ofstream::out);
		for (int i = 0; i < final.cols(); i++)
		{
			outfile << final(6, i) << endl;
		}
		*/

		vector <MatrixXd> results;
		vector < MatrixXd> alanes;
		int width = 1;
		double heading = final(6, 0);
		MatrixXd temp;
		for (int i = 1; i < final.cols(); i++)
		{
			double heading_l = final(6, i);
			if (heading < 0)
			{
				heading = 360 + heading;
			}
			if (heading_l < 0)
			{
				heading_l = 360 + heading_l;
			}
			if (abs(heading_l - heading) < 30)
			{
				temp.conservativeResize(7, width);
				temp.block(0, width - 1, 7, 1) = final.col(i);
				width++;
			}
			else
			{
				if (temp.cols() == 1)
				{
					heading = final(6, i);
					continue;
				}
				else
				{
					alanes.push_back(temp);
					width = 1;
					temp.resize(7, 1);
					heading = final(6, i);
				}
			}
		}
		if (temp.cols() != 1)
		{
			alanes.push_back(temp);
		}
		long long int count = 0;
		for (int j = 1; j < alanes.size() - 1; j++)
		{
			count = max(count, alanes[j].cols());
		}
		/*int fraction = 0;
		long long int count_c = count;
		while (count_c / 10 != 0)
		{
			fraction++;
			count_c = count_c / 10;
		}*/
		for (int i = 1; i < alanes.size() - 1; i++)
		{
			if (fabs(count - alanes[i].cols()) < (count * 1 / 2))
			{
				results.push_back(alanes[i]);
			}
		}
		return results;
	}

	/*对imu数据进行分离航带操作*/
	vector <MatrixXd> Aimudetach_2(vector < Matrix<double, 7, 2000>> data, int total, int rest)
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

		/*
		ofstream outfile("heading2.txt", ofstream::out);
		for (int i = 0; i < final.cols(); i++)
		{
			outfile << final(6, i) << endl;
		}
		*/

		vector <MatrixXd> results;
		vector < MatrixXd> alanes;
		int width = 1;
		double heading = final(6, 0);
		MatrixXd temp;
		for (int i = 1; i < final.cols(); i++)
		{
			double heading_l = final(6, i);
			if (heading < 0)
			{
				heading = 360 + heading;
			}
			if (heading_l < 0)
			{
				heading_l = 360 + heading_l;
			}
			if (abs(heading_l - heading) < 30)
			{
				temp.conservativeResize(7, width);
				temp.block(0, width - 1, 7, 1) = final.col(i);
				width++;
			}
			else
			{
				if (temp.cols() == 1)
				{
					heading = final(6, i);
					continue;
				}
				else
				{
					alanes.push_back(temp);
					width = 1;
					temp.resize(7, 1);
					heading = final(6, i);
				}
			}
		}
		if (temp.cols() != 1)
		{
			alanes.push_back(temp);
		}
		long long int count = 0;
		for (int j = 0; j < alanes.size(); j++)
		{
			count = max(count, alanes[j].cols());
		}
		for (int i = 0; i < alanes.size(); i++)
		{
			if (alanes[i].cols() > (count / 4))
			{
				results.push_back(alanes[i]);
			}
		}
		return results;
	}

	/*将坐标转换为绝对坐标系下*/
	void Act(Matrix<double, 6, Dynamic >& result_spilt, double xx, double yy, double zz, double r, double p, double h, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z)
	{
		//以下准备做安置
		result_spilt = anzhiMatrix * result_spilt;

		//以下准备做平移
		Parallel(result_spilt, CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);

		//以下准备做旋转
		GetRotationMatrix(r, p, h);
		result_spilt = RotationMatrix * result_spilt;

		GetRwMatrix(xx, yy);
		result_spilt = Rw * result_spilt;


		vector<double>X_GPS = LLA2ECEF(xx, yy, zz);
		for (int i = 0; i < result_spilt.cols(); i++)
		{
			double GPSeast, GPSnorth;
			int zone_number;
			string zone_letter;
			result_spilt(1, i) += X_GPS[0];
			result_spilt(2, i) += X_GPS[1];
			result_spilt(3, i) += X_GPS[2];
			vector<double>W_GPS = ECEF2LLA(result_spilt(1, i), result_spilt(2, i), result_spilt(3, i));
			if (from_latlon(W_GPS[0], W_GPS[1], GPSeast, GPSnorth, zone_number, zone_letter))
			{
				result_spilt(1, i) = GPSeast;
				result_spilt(2, i) = GPSnorth;
				result_spilt(3, i) = W_GPS[2];
			}
			else
			{
				throw"经纬度投影成UTM异常";
			}
		}
		/*
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
		*/
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

	void AGet_anzhi_RotationMatrix(double R, double P, double H)		//得到安置矩阵
	{
		if (R == 0 && P == 0 && H == 0)
		{
			for (int i = 0; i < 6; i++)
				for (int j = 0; j < 6; j++)
					if (i == j)anzhiMatrix(i, j) = 1;
					else anzhiMatrix(i, j) = 0;
		}
		else
		{
			for (int i = 0; i < 6; i++)
				for (int j = 0; j < 6; j++)
					if (i == j)anzhiMatrix(i, j) = 1;
					else anzhiMatrix(i, j) = 0;
			anzhiMatrix(1, 2) = -(H);
			anzhiMatrix(1, 3) = (P);
			anzhiMatrix(2, 1) = (H);
			anzhiMatrix(2, 3) = -(R);
			anzhiMatrix(3, 1) = -(P);
			anzhiMatrix(3, 2) = (R);
		}
	}

	void GetRwMatrix(double Lat, double Lon)   //得到Rw矩阵
	{
		double lat = radian(Lat);
		double lon = radian(Lon);
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				if (i == j)Rw(i, j) = 1;
				else Rw(i, j) = 0;
		Rw(1, 1) = -sin(lat) * cos(lon);
		Rw(1, 2) = -sin(lon);
		Rw(1, 3) = -cos(lat) * cos(lon);
		Rw(2, 1) = -sin(lat) * sin(lon);
		Rw(2, 2) = cos(lon);
		Rw(2, 3) = -cos(lat) * sin(lon);
		Rw(3, 1) = cos(lat);
		Rw(3, 2) = 0;
		Rw(3, 3) = -sin(lat);
	}

	void Parallel(Matrix<double, 6, Dynamic >& result_spilt, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z)     //矩阵平移
	{
		int t = result_spilt.cols();
		double line1, line2, line3;
		for (int i = 0; i < t; i++)
		{
			line1 = result_spilt(1, i);
			line2 = result_spilt(2, i);
			line3 = result_spilt(3, i);
			result_spilt(1, i) = CarrierToLidar_x - line2;
			result_spilt(2, i) = CarrierToLidar_y - line3;
			result_spilt(3, i) = CarrierToLidar_z + line1;
		}
	}

	vector<double>LLA2ECEF(double Lat, double Lon, double Alt)
	{
		double lat = radian(Lat);
		double lon = radian(Lon);
		vector<double>results;
		double a = 6378137.0;
		double b = 6356752.3142;
		//double f = 1 / 298.257223565;
		double e2 = (a * a - b * b) / (a * a);
		double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
		double X = (N + Alt) * cos(lat) * cos(lon);
		double Y = (N + Alt) * cos(lat) * sin(lon);
		double Z = (N * (1 - e2) + Alt) * sin(lat);
		results.push_back(X);
		results.push_back(Y);
		results.push_back(Z);
		return results;
	}

	vector<double>ECEF2LLA(double X, double Y, double Z)
	{
		double L;
		vector<double>results;
		double a = 6378137.0;
		double b = 6356752.3142;
		double e = sqrt((pow(a, 2) - pow(b, 2)) / pow(a, 2));
		if ((X == 0) && (Y > 0))
		{
			L = 90;
		}
		else if ((X == 0) && (Y < 0))
		{
			L = -90;
		}
		else if ((X < 0) && (Y >= 0))
		{
			L = atan2(Y, X);
			L = rad2angle(L);
			if (L < 0)
			{
				L = L + 180;
			}
		}
		else if ((X < 0) && (Y <= 0))
		{
			L = atan2(Y, X);
			L = rad2angle(L);
			L = L - 180;
		}
		else
		{
			L = atan2(Y, X);
			L = rad2angle(L);
		}


		double b0 = atan(Z / sqrt(pow(X, 2) + pow(Y, 2)));
		double N_temp = a / sqrt((1 - e * e * sin(b0) * sin(b0)));
		double b1 = atan((Z + N_temp * e * e * sin(b0)) / sqrt(pow(X, 2) + pow(Y, 2)));

		while (abs(b0 - b1) > 1e-7)
		{
			b0 = b1;
			N_temp = a / sqrt((1 - e * e * sin(b0) * sin(b0)));
			b1 = atan((Z + N_temp * e * e * sin(b0)) / sqrt(pow(X, 2) + pow(Y, 2)));
		}


		double B = b1;
		double N = a / sqrt((1 - e * e * sin(B) * sin(B)));
		double H = sqrt(pow(X, 2) + pow(Y, 2)) / cos(B) - N;
		B = rad2angle(B);
		results.push_back(B);
		results.push_back(L);
		results.push_back(H);
		return results;
	}

	double rad2angle(double r)
	{
		double a = r * 180.0 / M_PI;
		return a;
	}

	double angle2rad(double a)
	{
		double r = a * M_PI / 180.0;
		return r;
	}

	/*imu与lidar进行时间匹配 列表形式*/
	void Atime_match_m_l(MatrixXd& imu, list<Matrix<double, 6, Dynamic>>& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z)
	{
		list<Matrix<double, 6, Dynamic>>::iterator lidarpin = lidar.begin();
		int cols = imu.cols();
		int i = 0;//循环到第几个
		list<Matrix<double, 6, Dynamic>>::iterator temp;
		list<Matrix<double, 6, Dynamic>> listtemp;
		while (lidarpin != lidar.end() && i < cols)
		{
			if ((*lidarpin)(0, 0) < imu(0, 0))
			{
				temp = lidarpin;
				lidarpin++;
				listtemp.splice(listtemp.begin(), lidar, temp);
				listtemp.clear();
				continue;
			}
			if (fabs((*lidarpin)(0, 0) - imu(0, i)) < 1e-3)
			{

				Act((*lidarpin), imu(1, i), imu(2, i), imu(3, i), imu(4, i), imu(5, i), imu(6, i), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
				lidarpin++;
			}
			else if ((*lidarpin)(0, 0) > imu(0, i))
			{
				if ((*lidarpin)(0, 0) > imu(0, cols - 1))
				{
					temp = lidarpin;
					lidarpin++;
					listtemp.splice(listtemp.begin(), lidar, temp);
					listtemp.clear();
					continue;
				}
				i++;
				if ((*lidarpin)(0, 0) < imu(0, i))
				{
					i--;
					if (fabs((*lidarpin)(0, 0) - imu(0, i)) > 1e-3 && fabs((*lidarpin)(0, 0) - imu(0, i)) < 2 * 1e-3)
					{
						Act((*lidarpin), imu(1, i), imu(2, i), imu(3, i), imu(4, i), imu(5, i), imu(6, i), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
						i++;
						lidarpin++;
					}
					else
					{
						temp = lidarpin;
						lidarpin++;
						i++;
						listtemp.splice(listtemp.begin(), lidar, temp);
						listtemp.clear();
					}
				}
			}
			else
			{
				i--;
				if ((*lidarpin)(0, 0) > imu(0, i))
				{
					i++;
					if (fabs((*lidarpin)(0, 0) - imu(0, i)) > 1e-3 && fabs((*lidarpin)(0, 0) - imu(0, i)) < 2 * 1e-3)
					{
						Act((*lidarpin), imu(1, i), imu(2, i), imu(3, i), imu(4, i), imu(5, i), imu(6, i), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
						lidarpin++;
					}
					else
					{
						temp = lidarpin;
						lidarpin++;
						listtemp.splice(listtemp.begin(), lidar, temp);
						listtemp.clear();
					}
				}
			}
		}
	}

	/*imu与lidar进行时间匹配同时生成txt文件 列表形式*/
	void Atime_match_m_l_txt(MatrixXd& imu, list<Matrix<double, 6, Dynamic>>& lidar, double CarrierToLidar_x, double CarrierToLidar_y, double CarrierToLidar_z, int number)
	{
		string filename = "数据" + to_string(number) + ".txt";
		ofstream outFile(filename, ios::out);
		if (!outFile)
		{
			throw "创建文件失败！";
		}
		outFile << "timetamp  original_x  original_y  original_z  number  intensity  Lat Lon  Alti  h  p  r  result_x  result_y  result_z\n";
		list<Matrix<double, 6, Dynamic>>::iterator lidarpin = lidar.begin();
		int cols = imu.cols();
		int i = 0;//循环到第几个
		list<Matrix<double, 6, Dynamic>>::iterator temp;
		list<Matrix<double, 6, Dynamic>> listtemp;
		while (lidarpin != lidar.end() && i < cols)
		{
			if ((*lidarpin)(0, 0) < imu(0, 0))
			{
				temp = lidarpin;
				lidarpin++;
				listtemp.splice(listtemp.begin(), lidar, temp);
				listtemp.clear();
				continue;
			}
			if (fabs((*lidarpin)(0, 0) - imu(0, i)) < 1e-3)
			{
				Matrix<double, 6, Dynamic> temp_lm = (*lidarpin).block(0, 0, (*lidarpin).rows(), (*lidarpin).cols());
				Act((*lidarpin), imu(1, i), imu(2, i), imu(3, i), imu(4, i), imu(5, i), imu(6, i), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
				for (int j = 0; j < (*lidarpin).cols(); j++)
				{
					outFile << setprecision(14) << (*lidarpin)(0, j) << "\t" << temp_lm(1, j) << "\t" << temp_lm(2, j) << "\t" << temp_lm(3, j) << "\t" << (*lidarpin)(5, j) << "\t" << (*lidarpin)(4, j) << "\t" << imu(1, i) << "\t" << imu(2, i) << "\t" << imu(3, i) << "\t" << imu(6, i) << "\t" << imu(5, i) << "\t" << imu(4, i) << "\t" << (*lidarpin)(1, j) << "\t" << (*lidarpin)(2, j) << "\t" << (*lidarpin)(3, j) << '\n';
				}
				lidarpin++;
			}
			else if ((*lidarpin)(0, 0) > imu(0, i))
			{
				if ((*lidarpin)(0, 0) > imu(0, cols - 1))
				{
					temp = lidarpin;
					lidarpin++;
					listtemp.splice(listtemp.begin(), lidar, temp);
					listtemp.clear();
					continue;
				}
				i++;
				if ((*lidarpin)(0, 0) < imu(0, i))
				{
					i--;
					if (fabs((*lidarpin)(0, 0) - imu(0, i)) > 1e-3 && fabs((*lidarpin)(0, 0) - imu(0, i)) < 2 * 1e-3)
					{
						Matrix<double, 6, Dynamic> temp_lm = (*lidarpin).block(0, 0, (*lidarpin).rows(), (*lidarpin).cols());
						Act((*lidarpin), imu(1, i), imu(2, i), imu(3, i), imu(4, i), imu(5, i), imu(6, i), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
						for (int j = 0; j < (*lidarpin).cols(); j++)
						{
							outFile << setprecision(14) << (*lidarpin)(0, j) << "\t" << temp_lm(1, j) << "\t" << temp_lm(2, j) << "\t" << temp_lm(3, j) << "\t" << (*lidarpin)(5, j) << "\t" << (*lidarpin)(4, j) << "\t" << imu(1, i) << "\t" << imu(2, i) << "\t" << imu(3, i) << "\t" << imu(6, i) << "\t" << imu(5, i) << "\t" << imu(4, i) << "\t" << (*lidarpin)(1, j) << "\t" << (*lidarpin)(2, j) << "\t" << (*lidarpin)(3, j) << '\n';
						}
						i++;
						lidarpin++;
					}
					else
					{
						temp = lidarpin;
						lidarpin++;
						i++;
						listtemp.splice(listtemp.begin(), lidar, temp);
						listtemp.clear();
					}
				}
			}
			else
			{
				i--;
				if ((*lidarpin)(0, 0) > imu(0, i))
				{
					i++;
					if (fabs((*lidarpin)(0, 0) - imu(0, i)) > 1e-3 && fabs((*lidarpin)(0, 0) - imu(0, i)) < 2 * 1e-3)
					{
						Matrix<double, 6, Dynamic> temp_lm = (*lidarpin).block(0, 0, (*lidarpin).rows(), (*lidarpin).cols());
						Act((*lidarpin), imu(1, i), imu(2, i), imu(3, i), imu(4, i), imu(5, i), imu(6, i), CarrierToLidar_x, CarrierToLidar_y, CarrierToLidar_z);
						for (int j = 0; j < (*lidarpin).cols(); j++)
						{
							outFile << setprecision(14) << (*lidarpin)(0, j) << "\t" << temp_lm(1, j) << "\t" << temp_lm(2, j) << "\t" << temp_lm(3, j) << "\t" << (*lidarpin)(5, j) << "\t" << (*lidarpin)(4, j) << "\t" << imu(1, i) << "\t" << imu(2, i) << "\t" << imu(3, i) << "\t" << imu(6, i) << "\t" << imu(5, i) << "\t" << imu(4, i) << "\t" << (*lidarpin)(1, j) << "\t" << (*lidarpin)(2, j) << "\t" << (*lidarpin)(3, j) << '\n';
						}
						lidarpin++;
					}
					else
					{
						temp = lidarpin;
						lidarpin++;
						listtemp.splice(listtemp.begin(), lidar, temp);
						listtemp.clear();
					}
				}
			}
		}
		outFile.close();
	}
}







