#include "interface.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "Aviaprocess.h"
#include "IMUprocess.h"
#include "RSprocess.h"
#include "timematch.h"
#include "Write_pcd.h"
#include "Write_ply.h"
#include "Write_las.h"
#include <thread>
#include <ctime>


using namespace Las;
using namespace Ply;
using namespace Pcd;
using namespace IMUissue;
using namespace Lidarissue;
using namespace Aviaissue;
using namespace TIME;
using namespace Eigen;
using namespace std;
namespace Interface
{
	void process_n_n(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 5);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 20);
				int xxx = reading.size() / num;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(num);
				for (int i = 0; i < (num-1); i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[num-1].splice(reading1[num-1].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ�����ʼ��\n");
				(*ob.*settext)(ob, theword);
				vector<thread>ths;
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(lp.lidar_work_model, 16), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
					(*ob.*setvalue)(ob, int(20 + ccount * 20 / num));
				}
				ths.clear();
				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = (num-1); i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 50);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			for (int i = 0; i < total - 1; i++)
			{
				imufinal.conservativeResize(7, 2000 * (i + 1));
				imufinal.block(0, i * 2000, 7, 2000) = imuresult[i];
			}
			imufinal.conservativeResize(7, 2000 * (total - 1) + rest);
			imufinal.block(0, (total - 1) * 2000, 7, rest) = imuresult[total - 1].block(0, 0, 7, rest);
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 60);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 70);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 85);
			//�������ɵ����ļ�
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 90);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePcd_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePly_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 100);
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 5);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 20);
				int xxx = reading.size() / num;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(num);
				for (int i = 0; i < (num-1); i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[num - 1].splice(reading1[num - 1].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ�����ʼ��\n");
				(*ob.*settext)(ob, theword);
				vector<thread>ths;
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(lp.lidar_work_model, 32), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
					(*ob.*setvalue)(ob, int(20 + ccount * 20 / num));
				}
				ths.clear();
				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = (num-1); i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 50);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			for (int i = 0; i < total - 1; i++)
			{
				imufinal.conservativeResize(7, 2000 * (i + 1));
				imufinal.block(0, i * 2000, 7, 2000) = imuresult[i];
			}
			imufinal.conservativeResize(7, 2000 * (total - 1) + rest);
			imufinal.block(0, (total - 1) * 2000, 7, rest) = imuresult[total - 1].block(0, 0, 7, rest);
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 60);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 70);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 85);
			//�������ɵ����ļ�
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 90);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePcd_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePly_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 100);
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try 
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lidar_final = lvxReader(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 40);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			for (int i = 0; i < total - 1; i++)
			{
				imufinal.conservativeResize(7, 2000 * (i + 1));
				imufinal.block(0, i * 2000, 7, 2000) = imuresult[i];
			}
			imufinal.conservativeResize(7, 2000 * (total - 1) + rest);
			imufinal.block(0, (total - 1) * 2000, 7, rest) = imuresult[total - 1].block(0, 0, 7, rest);
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 50);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 55);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 60);
			time_match_m_m(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 80);
			//�������ɵ����ļ�
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 85);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_m(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePcd_m(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePly_m(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 100);
		}
		else
		{
			strcpy_s(theword, "�״������쳣��\n");
			(*ob.*settext)(ob, theword);
			return;
		}
	}

	void process_y_n(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 5);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 20);
				int xxx = reading.size() / num;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(num);
				for (int i = 0; i < (num-1); i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[num - 1].splice(reading1[num - 1].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ�����ʼ��\n");
				(*ob.*settext)(ob, theword);
				vector<thread>ths;
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(lp.lidar_work_model, 16), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
					(*ob.*setvalue)(ob, int(20 + ccount * 20 / num));
				}
				ths.clear();
				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = (num-1); i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 50);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 55);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 60);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			long long nums = 0;
			long long current = 0;
			for (int i = 0; i < results.size(); i++)
			{
				nums += results[i].cols();
				imufinal.conservativeResize(7, nums);
				imufinal.block(0, current, 7, results[i].cols()) = results[i];
				current += results[i].cols();
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();
			(*ob.*setvalue)(ob, 65);

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 70);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 75);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 85);
			//�������ɵ����ļ�
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 90);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePcd_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePly_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 100);
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 5);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 20);
				int xxx = reading.size() / num;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(num);
				for (int i = 0; i < (num-1); i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[num-1].splice(reading1[num-1].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ�����ʼ��\n");
				(*ob.*settext)(ob, theword);
				vector<thread>ths;
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(lp.lidar_work_model, 32), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
					(*ob.*setvalue)(ob, int(20 + ccount * 20 / num));
				}
				ths.clear();
				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = (num-1); i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 50);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 55);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 60);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			long long nums = 0;
			long long current = 0;
			for (int i = 0; i < results.size(); i++)
			{
				nums += results[i].cols();
				imufinal.conservativeResize(7, nums);
				imufinal.block(0, current, 7, results[i].cols()) = results[i];
				current += results[i].cols();
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();
			(*ob.*setvalue)(ob, 65);

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 70);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 75);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 85);
			//�������ɵ����ļ�
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 90);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePcd_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePly_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 100);
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lidar_final = lvxReader(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 40);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 45);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 50);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			long long nums = 0;
			long long current = 0;
			for (int i = 0; i < results.size(); i++)
			{
				nums += results[i].cols();
				imufinal.conservativeResize(7, nums);
				imufinal.block(0, current, 7, results[i].cols()) = results[i];
				current += results[i].cols();
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();
			(*ob.*setvalue)(ob, 60);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 70);
			time_match_m_m(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 85);
			//�������ɵ����ļ�
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 90);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_m(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePcd_m(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePly_m(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 100);
		}
		else
		{
			strcpy_s(theword, "�״������쳣��\n");
			(*ob.*settext)(ob, theword);
			return;
		}
	}

	void process_y_y(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 5);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 20);
				int xxx = reading.size() / num;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(num);
				for (int i = 0; i < (num-1); i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[num-1].splice(reading1[num-1].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ�����ʼ��\n");
				(*ob.*settext)(ob, theword);
				vector<thread>ths;
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(lp.lidar_work_model, 16), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
					(*ob.*setvalue)(ob, int(20 + ccount * 20 / num));
				}
				ths.clear();
				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = (num-1); i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 45);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 50);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 55);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			(*ob.*setvalue)(ob, 65);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 70);
			int cccount = results.size();
			for (int i = 0; i < results.size(); i++)
			{
				list<Matrix<double, 6, Dynamic>> temp;
				list<Matrix<double, 6, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}

				//���½���imu��lidarʱ��ƥ��
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				string sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcat_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
				time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 15 / cccount));
				if (temp.size() == 0)
				{
					strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
					(*ob.*settext)(ob, theword);
					continue;
				}
				//�������ɵ����ļ�
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				sen = "]���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcat_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 20 / cccount));
				if (fout.points_file_type == 0)
				{
					string file = fout.points_file + to_string(i + 1) + ".las";
					writeLas_l(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					writePcd_l(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					writePly_l(file, temp);
				}
				else
				{
					strcpy_s(theword, "����ļ�����ѡ�����\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]�������ļ�������ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 30 / cccount));
				temp.clear();
			}
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 5);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 20);
				int xxx = reading.size() / num;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(num);
				for (int i = 0; i < (num-1); i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[num-1].splice(reading1[num-1].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ�����ʼ��\n");
				(*ob.*settext)(ob, theword);
				vector<thread>ths;
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(lp.lidar_work_model, 32), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
					(*ob.*setvalue)(ob, int(20 + ccount * 20 / num));
				}
				ths.clear();


				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = (num-1); i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 45);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 50);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 55);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			(*ob.*setvalue)(ob, 65);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 70);
			int cccount = results.size();
			for (int i = 0; i < results.size(); i++)
			{
				list<Matrix<double, 6, Dynamic>> temp;
				list<Matrix<double, 6, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				//���½���imu��lidarʱ��ƥ��
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				string sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcat_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
				time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 15 / cccount));
				(*ob.*settext)(ob, theword);
				if (temp.size() == 0)
				{
					strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
					(*ob.*settext)(ob, theword);
					continue;
				}
				//�������ɵ����ļ�
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				sen = "]���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcat_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 20 / cccount));
				if (fout.points_file_type == 0)
				{
					string file = fout.points_file + to_string(i + 1) + ".las";
					writeLas_l(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					writePcd_l(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					writePly_l(file, temp);
				}
				else
				{
					strcpy_s(theword, "����ļ�����ѡ�����\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]�������ļ�������ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(70 + (i + 1) * 30 / cccount));
				temp.clear();
			}
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lidar_final = lvxReader(fin.lidar_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 35);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 40);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			(*ob.*setvalue)(ob, 45);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			(*ob.*setvalue)(ob, 60);
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			int cccount = results.size();
			for (int i = 0; i < results.size(); i++)
			{
				MatrixXd temp;
				temp.resize(lidar_final.rows(), lidar_final.cols());
				temp.block(0, 0, lidar_final.rows(), lidar_final.cols()) = lidar_final;
				//���½���imu��lidarʱ��ƥ��
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				string sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcat_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(65 + (i + 1) * 5 / cccount));
				time_match_m_m(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(65 + (i + 1) * 15 / cccount));
				if (temp.cols() == 0)
				{
					strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
					(*ob.*settext)(ob, theword);
					continue;
				}
				//�������ɵ����ļ�
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				sen = "]���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcat_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(65 + (i + 1) * 20 / cccount));
				if (fout.points_file_type == 0)
				{
					string file = fout.points_file + to_string(i + 1) + ".las";
					writeLas_m(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					writePcd_m(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					writePly_m(file, temp);
				}
				else
				{
					strcpy_s(theword, "����ļ�����ѡ�����\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]�������ļ�������ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, int(65 + (i + 1) * 35 / cccount));
			}
		}
		else
		{
			strcpy_s(theword, "�״������쳣��\n");
			(*ob.*settext)(ob, theword);
			return;
		}
	}
}