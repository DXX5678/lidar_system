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
#define PI 3.1415926
using namespace SplineSpace;
using namespace Eigen;
using namespace std;
using namespace conversion;
namespace IMUissue
{
	/*����С�����nλ����*/
	double precision(double x, int y)
	{
		double temp = pow(0.1, y + 1);
		x += temp * 5;
		int temp2 = pow(10, y);
		long long int m = floor(x * temp2);
		x = (double)m / temp2;
		return x;
	}
	/*��ȡ������imu��ʼ���ݰ�*/
	vector < Matrix<double, 7, 2000>>  Read(string name, int& total, int& rest)
	{
		vector < Matrix<double, 7, 2000>>data;
		ifstream inFile(name, ios::in | ios::binary);
		if (!inFile)
		{
			throw "�ļ���ʧ�ܣ�";
		}
		while (inFile.peek() != EOF)
		{
			Matrix<double, 7, 2000> temp_matrix;//���ɾ�������ѹ������
			temp_matrix = temp_matrix.Zero();
			for (int i = 0; i < numbers;)
			{
				double temp[7];//������������װ����ɾ���
				for (int j = 0; j < 7; j++)
				{
					if (j == 4)inFile.seekg(3 * 8, ios::cur);
					inFile.read((char*)&temp[j], 8);
					if (j == 6)inFile.seekg(7 * 8, ios::cur);
				}
				temp[1] = (temp[1] * 180) / PI;
				temp[2] = (temp[2] * 180) / PI;
				temp[4] = (temp[4] * 180) / PI;
				temp[5] = (temp[5] * 180) / PI;
				temp[6] = (temp[6] * 180) / PI;
				double GPSeast, GPSnorth;
				int zone_number;
				string zone_letter;
				if (from_latlon(temp[1], temp[2], GPSeast, GPSnorth, zone_number, zone_letter))//�ж���ȷͶӰ
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

				}
			}
			data.push_back(temp_matrix);
			total++;//��¼�ܵľ�������
		}
		inFile.close();
		return data;
	}
	/*����imu���ݲ�ֵ(����������ֵ)*/
	void imu_timenew(vector < Matrix<double, 7, 2000>>& data, int total, int rest)
	{
		int sub = precision((data[0](0, 1) - data[0](0, 0)) / 0.001, 0);//����
		for (int m = 0; m < total - 1; m++)//��n-1�ž�����б���
		{
			double time_result[5 * numbers];
			for (int i = 0; i < numbers; i++)
				for (int j = 0; j <= sub - 1; j++)
				{
					time_result[i * sub + j] = data[m](0, i) + j * 0.001;//ȷ��������ֵʱ��
				}
			double x[numbers + 1];
			for (int i = 0; i < numbers; i++)
				x[i] = data[m](0, i);//ȷ������ʱ���
			x[numbers] = data[m + 1](0, 0);
			for (int i = 0; i < numbers * 5; i++)
			{
				data[m](0, i) = time_result[i];
			}
			for (int k = 1; k <= 6; k++)//��������Ҫ��ֵ�Ĳ�������ѭ����ֵ
			{
				double y[numbers + 1];
				for (int i = 0; i < numbers; i++)
					y[i] = data[m](k, i);
				y[numbers] = data[m + 1](k, 0);
				double leftBound = 0, RightBound = 0;	//�߽絼��
				Spline sp(x, y, numbers + 1, GivenSecondOrder, leftBound, RightBound);
				double y_result[5 * numbers];
				sp.MultiPointInterp(time_result, 5 * numbers, y_result);			//��x�Ĳ�ֵ���y
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
				time_result[i * sub + j] = data[total - 1](0, i) + j * 0.001;//ȷ��������ֵʱ��
				if (i == rest - 1)break;
			}
		double x[numbers];
		for (int i = 0; i < rest; i++)
			x[i] = data[total - 1](0, i);//ȷ������ʱ���
		for (int i = 0; i < rest * 5 - 4; i++)
		{
			data[total - 1](0, i) = time_result[i];
		}
		for (int k = 1; k <= 6; k++)//��������Ҫ��ֵ�Ĳ�������ѭ����ֵ
		{
			double y[numbers];
			for (int i = 0; i < rest; i++)
				y[i] = data[total - 1](k, i);
			double leftBound = 0, RightBound = 0;	//�߽絼��
			Spline sp(x, y, rest, GivenSecondOrder, leftBound, RightBound);
			double y_result[5 * numbers - 4];
			sp.MultiPointInterp(time_result, 5 * rest - 4, y_result);			//��x�Ĳ�ֵ���y
			for (int i = 0; i < 5 * rest - 4; i++)
			{
				data[total - 1](k, i) = y_result[i];
			}
		}
	}
}