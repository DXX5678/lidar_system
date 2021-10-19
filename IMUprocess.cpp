#include <iostream>
#include <fstream>
#include "Spline.h"
#include<windows.h>
#include<iomanip>
#include <algorithm>
#include <corecrt_math_defines.h>
#include <math.h>
#include "IMUprocess.h"
#include "IMUanalyze.h"
#define _USE_MATH_DEFINES
#define numbers 400
#define result_numbers numbers*5
#define PI 3.1415926
using namespace SplineSpace;
using namespace Eigen;
using namespace std;
using namespace conversion;
namespace IMUissue
{
	int total;
	int rest;

	/*保留小数点后n位操作*/
	double precision(double x, int y)
	{
		double temp = pow(0.1, y + 1);
		x += temp * 5;
		int temp2 = pow(10, y);
		long long int m = floor(x * temp2);
		x = (double)m / temp2;
		return x;
	}
	/*读取并解析imu初始数据包*/
	vector < Matrix<double, 7, 2000>>  Read(string name)
	{

		//ofstream outfile("heading2.txt", ofstream::out);

		total = 0;
		rest = 0;
		vector < Matrix<double, 7, 2000>>data;
		ifstream inFile(name, ios::in | ios::binary);
		if (!inFile)
		{
			throw "文件打开失败！";
		}
		while (inFile.peek() != EOF)
		{
			Matrix<double, 7, 2000> temp_matrix;//过渡矩阵用于压进容器
			temp_matrix = temp_matrix.Zero();
			for (int i = 0; i < numbers;)
			{
				double temp[8];//过渡数组用于装入过渡矩阵
				for (int j = 0; j < 8; j++)
				{
					if (j == 4)inFile.seekg(3 * 8, ios::cur);
					inFile.read((char*)&temp[j], 8);
					if (j == 7)inFile.seekg(6 * 8, ios::cur);
				}
				temp[1] = (temp[1] * 180) / PI;
				temp[2] = (temp[2] * 180) / PI;
				temp[4] = (temp[4] * 180) / PI;
				temp[5] = (temp[5] * 180) / PI;
				temp[6] = (temp[6] * 180) / PI;
				temp[7] = (temp[7] * 180) / PI;
				temp[6] = temp[6] - temp[7];
				//outfile << temp[4] << endl;

				//double GPSeast, GPSnorth;
				//int zone_number;
				//string zone_letter;
				//temp[1] = precision(temp[1], 8);
				//temp[2] = precision(temp[2], 8);
				/*
				if (from_latlon(temp[1], temp[2], GPSeast, GPSnorth, zone_number, zone_letter))//判定正确投影
				{
					temp[2] = GPSeast;
					temp[1] = GPSnorth;
					for (int j = 0; j < 7; j++)
					{
						if (j < 4)temp_matrix(j, i) = precision(temp[j], 3);
						else temp_matrix(j, i) = precision(temp[j], 9);
					}
					i++;
				}
				else
				{
					if (inFile.peek() == EOF)
					{
						rest = i;
						data.push_back(temp_matrix);
						break;
					}

				}*/
				for (int j = 0; j < 7; j++)
				{
					if (j == 0)temp_matrix(j, i) = precision(temp[j], 3);
					else if (j < 4)temp_matrix(j, i) = precision(temp[j], 9);
					else temp_matrix(j, i) = precision(temp[j], 9);
				}
				i++;
				if (inFile.peek() == EOF)
				{
					rest = i;
					break;
				}
			}
			data.push_back(temp_matrix);
			total++;//记录总的矩阵数量
		}
		inFile.close();
		return data;
	}
	/*读取并解析imu初始数据包版本2*/
	vector < Matrix<double, 7, 2000>>  Read_2(string name)
	{

		//ofstream outfile("heading2.txt", ofstream::out);

		total = 0;
		rest = 0;
		vector < Matrix<double, 7, 2000>>data;
		ifstream inFile(name, ios::in | ios::binary);
		if (!inFile)
		{
			throw "文件打开失败！";
		}
		while (inFile.peek() != EOF)
		{
			Matrix<double, 7, 2000> temp_matrix;//过渡矩阵用于压进容器
			temp_matrix = temp_matrix.Zero();
			for (int i = 0; i < numbers;)
			{
				double temp[10];//过渡数组用于装入过渡矩阵
				for (int j = 0; j < 10; j++)
				{
					if (j == 4)inFile.seekg(2 * 8, ios::cur);
					if (j == 9)inFile.seekg(2 * 8, ios::cur);
					inFile.read((char*)&temp[j], 8);
					if (j == 9)inFile.seekg(3 * 8, ios::cur);
				}
				temp[1] = (temp[1] * 180) / PI;
				temp[2] = (temp[2] * 180) / PI;
				temp[5] = (temp[5] * 180) / PI;
				temp[6] = (temp[6] * 180) / PI;
				temp[7] = (temp[7] * 180) / PI;
				temp[8] = (temp[8] * 180) / PI;
				temp[7] = temp[7] - temp[8];
				/*if (abs(temp[4]) > 1 || abs(temp[9]) > 1)
				{
					continue;
				}*/
				//outfile << temp[4] << endl;

				double GPSeast, GPSnorth;
				int zone_number;
				string zone_letter;
				temp[1] = precision(temp[1], 9);
				temp[2] = precision(temp[2], 9);
				
				if (from_latlon(temp[1], temp[2], GPSeast, GPSnorth, zone_number, zone_letter))//判定正确投影
				{
					temp[2] = GPSeast;
					temp[1] = GPSnorth;
					for (int j = 0; j < 7; j++)
					{
						if (j == 0)temp_matrix(j, i) = precision(temp[j], 3);
						else if (j < 4)temp_matrix(j, i) = precision(temp[j], 9);
						else temp_matrix(j, i) = precision(temp[j], 9);
					}
					i++;
				}
				else
				{
					if (inFile.peek() == EOF)
					{
						rest = i;
						//data.push_back(temp_matrix);
						break;
					}

				}
				/*for (int j = 0; j < 7; j++)
				{
					if (j == 0)temp_matrix(j, i) = precision(temp[j], 3);
					else if (j < 4)temp_matrix(j, i) = precision(temp[j], 9);
					else temp_matrix(j, i) = precision(temp[j + 1], 9);
				}
				i++;*/
				if (inFile.peek() == EOF)
				{
					rest = i;
					break;
				}
			}
			data.push_back(temp_matrix);
			total++;//记录总的矩阵数量
		}
		inFile.close();
		return data;
	}
	/*进行imu数据插值(三次样条插值)*/
	void imu_timenew(vector < Matrix<double, 7, 2000>>& data)
	{
		int sub = precision((data[0](0, 1) - data[0](0, 0)) / 0.001, 0);//求差距
		for (int m = 0; m < total - 1; m++)//对n-1张矩阵进行遍历
		{
			double time_result[5 * numbers];
			for (int i = 0; i < numbers; i++)
				for (int j = 0; j <= sub - 1; j++)
				{
					time_result[i * sub + j] = data[m](0, i) + j * 0.001;//确定基础差值时间
				}
			double x[numbers + 1];
			for (int i = 0; i < numbers; i++)
				x[i] = data[m](0, i);//确定现有时间戳
			x[numbers] = data[m + 1](0, 0);
			for (int i = 0; i < numbers * 5; i++)
			{
				data[m](0, i) = time_result[i];
			}
			for (int k = 1; k <= 6; k++)//对六个需要差值的参数进行循环差值
			{
				double y[numbers + 1];
				for (int i = 0; i < numbers; i++)
					y[i] = data[m](k, i);
				y[numbers] = data[m + 1](k, 0);
				double leftBound = 0, RightBound = 0;	//边界导数
				Spline sp(x, y, numbers + 1, GivenSecondOrder, leftBound, RightBound);
				double y_result[5 * numbers];
				sp.MultiPointInterp(time_result, 5 * numbers, y_result);			//求x的插值结果y
				for (int i = 0; i < 5 * numbers; i++)
				{
					data[m](k, i) = y_result[i];
				}
			}
		}
		double time_result[5 * numbers];
		for (int i = 0; i < rest; i++)
			for (int j = 0; j <= sub - 1; j++)
			{
				time_result[i * sub + j] = data[total - 1](0, i) + j * 0.001;//确定基础差值时间
				if (i == rest - 1)break;
			}
		double x[numbers];
		for (int i = 0; i < rest; i++)
			x[i] = data[total - 1](0, i);//确定现有时间戳
		for (int i = 0; i < rest * 5 - 4; i++)
		{
			data[total - 1](0, i) = time_result[i];
		}
		for (int k = 1; k <= 6; k++)//对六个需要差值的参数进行循环差值
		{
			double y[numbers];
			for (int i = 0; i < rest; i++)
				y[i] = data[total - 1](k, i);
			double leftBound = 0, RightBound = 0;	//边界导数
			Spline sp(x, y, rest, GivenSecondOrder, leftBound, RightBound);
			double y_result[5 * numbers - 4];
			sp.MultiPointInterp(time_result, 5 * rest - 4, y_result);			//求x的插值结果y
			for (int i = 0; i < 5 * rest - 4; i++)
			{
				data[total - 1](k, i) = y_result[i];
			}
		}
	}

	/*角度转换*/
	double transition(double x, double y, double z, int h)
	{
		double temp = x + (double)y / 60.0 + (double)z / 3600.0;
		return precision(temp, h);
	}

	/*读imu数据txt版本*/
	vector < Matrix<double, 7, 2000>>  Read_txt(string name)
	{
		//ofstream outfile("heading.txt", ofstream::out);
		vector < Matrix<double, 7, 2000>>data;
		ifstream inFile(name, ifstream::in);
		string str0;
		int nn = 0;
		Matrix<double, 7, 2000> temp_matrix;//过渡矩阵用于压进容器
		temp_matrix = temp_matrix.Zero();
		while (getline(inFile, str0))
		{
			istringstream strin;
			strin.str(str0);
			string token;
			double temp_array[11];
			int temp_n = 0;
			int chenchentrue = 1;
			while (getline(strin, token, ' '))
			{
				double temp = atof(token.c_str());
				temp_array[temp_n] = temp;
				temp_n++;
				if (temp_n >= 11)break;

			}
			int haha = 1;
			for (int i = 0; i <= 10; i++)
			{
				if (fabs(temp_array[i] - 0) < 1e-6) { haha = 0; break; }
			}
			if (haha == 0)continue;


			temp_matrix(0, nn) = precision(temp_array[0], 3);
			temp_matrix(1, nn) = transition(temp_array[1], temp_array[2], temp_array[3], 9);
			temp_matrix(2, nn) = transition(temp_array[4], temp_array[5], temp_array[6], 9);
			temp_matrix(3, nn) = precision(temp_array[7], 3);
			temp_matrix(4, nn) = precision(temp_array[10], 4);
			temp_matrix(5, nn) = precision(temp_array[9], 4);
			temp_matrix(6, nn) = precision(temp_array[8], 4);

			//outfile << temp_matrix(4, nn) << endl;
			nn++;
			if (nn == result_numbers)
			{
				data.push_back(temp_matrix);
				nn = 0;
				total++;
			}
		}
		data.push_back(temp_matrix);
		total++;
		rest = nn;
		inFile.close();
		return data;
	}
}