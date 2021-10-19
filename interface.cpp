#include "interface.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "Aviaprocess.h"
#include "IMUprocess.h"
#include "RSprocess.h"
#include "timematch.h"
#include "timematch_avia.h"
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
using namespace TIMEA;
using namespace Eigen;
using namespace std;
namespace Interface
{
	void process_n_n(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num, bool choice)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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
				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
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
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 60);
			}
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			if (!choice)
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 70);
					time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
			else
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䲢�������ݡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 70);
					time_match_m_l_txt(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɣ��������ݳɹ���\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
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
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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
				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
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
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 60);
			}
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			if (!choice)
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 70);
					time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
			else
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䲢�������ݡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 70);
					time_match_m_l_txt(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɣ��������ݳɹ���\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
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
			list<Matrix<double, 6, Dynamic>> lidar_final;
			try 
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lvxReader(fin.lidar_data_file, lidar_final);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
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
				strcat_s(theword, "]��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 50);
			}
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			AGet_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 55);
			if (!choice)
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 60);
					Atime_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 80);
			}
			else
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䲢�������ݡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 60);
					Atime_match_m_l_txt(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɣ��������ݳɹ���\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 80);
			}
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
				AwriteLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				AwritePcd_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				AwritePly_l(file, lidar_final);
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

	void process_y_n(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num, bool choice)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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
				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 55);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 60);
				vector < MatrixXd> results = Aimudetach(imuresult, total, rest);
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
			}
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 70);
			if (!choice)
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 75);
					time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䲢�������ݡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 75);
					time_match_m_l_txt(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɣ��������ݳɹ���\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
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
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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
				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 55);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 60);
				vector < MatrixXd> results = Aimudetach(imuresult, total, rest);
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
			}
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 70);
			if (!choice)
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 75);
					time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
			else
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䲢�������ݡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 75);
					time_match_m_l_txt(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɣ��������ݳɹ���\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
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
			list<Matrix<double, 6, Dynamic>> lidar_final;
			try
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lvxReader(fin.lidar_data_file, lidar_final);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 45);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 50);
				vector < MatrixXd> results = Aimudetach(imuresult, total, rest);
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
			}
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			AGet_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			if (!choice)
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䡣\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 70);
					Atime_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
			else
			{
				try
				{
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]����ʼʱ��ƥ�䲢�������ݡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 70);
					Atime_match_m_l_txt(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��ʱ��ƥ����ɣ��������ݳɹ���\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 85);
			}
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
				AwriteLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				AwritePcd_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				AwritePly_l(file, lidar_final);
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

	void process_y_y(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num, bool choice)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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
				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
				results = imudetach(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 50);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 55);
				results = Aimudetach(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
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
				list<Matrix<double, 7, Dynamic>> temp;
				list<Matrix<double, 7, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				string sen;
				//���½���imu��lidarʱ��ƥ��
				if (!choice)
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
				}
				else
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l_txt(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset, (i+1));
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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


				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
				results = imudetach(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 50);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 55);
				results = Aimudetach(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
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
				list<Matrix<double, 7, Dynamic>> temp;
				list<Matrix<double, 7, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				string sen;
				//���½���imu��lidarʱ��ƥ��
				if (!choice)
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
				}
				else
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l_txt(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset, (i+1));
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
			list<Matrix<double, 6, Dynamic>> lidar_final;
			try
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lvxReader(fin.lidar_data_file, lidar_final);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
				results = imudetach(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 60);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 40);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 45);
				results = Aimudetach(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 60);
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			AGet_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			int cccount = results.size();
			for (int i = 0; i < results.size(); i++)
			{
				list<Matrix<double, 6, Dynamic>> temp;
				list<Matrix<double, 6, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				string sen;
				//���½���imu��lidarʱ��ƥ��
				if (!choice)
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(65 + (i + 1) * 5 / cccount));
						Atime_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, int(65 + (i + 1) * 15 / cccount));
					if (temp.size() == 0)
					{
						strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						(*ob.*settext)(ob, theword);
						continue;
					}
				}
				else
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(65 + (i + 1) * 5 / cccount));
						Atime_match_m_l_txt(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset, (i+1));
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, int(65 + (i + 1) * 15 / cccount));
					if (temp.size() == 0)
					{
						strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						(*ob.*settext)(ob, theword);
						continue;
					}
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
					AwriteLas_l(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					AwritePcd_l(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					AwritePly_l(file, temp);
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

	void process_y_y_e(mythread* ob, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp, int num, bool choice)
	{
		char theword[100];
		time_t now;
		char tmp[16];
		(*ob.*setvalue)(ob, 0);
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				for (int i = 0; i < (num - 1); i++)
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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
				list<Matrix<double, 7, Dynamic>>::iterator pp;
				for (int i = (num - 1); i >= 0; i--)
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
				results = imudetach_2(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 50);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 55);
				results = Aimudetach_2(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
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
				list<Matrix<double, 7, Dynamic>> temp;
				list<Matrix<double, 7, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				string sen;
				//���½���imu��lidarʱ��ƥ��
				if (!choice)
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
				}
				else
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l_txt(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset, (i + 1));
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
			list<Matrix<double, 7, Dynamic>> lidar_final;
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
				for (int i = 0; i < (num - 1); i++)
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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


				list<Matrix<double, 7, Dynamic>>::iterator pp;
				for (int i = (num - 1); i >= 0; i--)
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
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
				results = imudetach_2(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 50);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 55);
				results = Aimudetach_2(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 65);
			}
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
				list<Matrix<double, 7, Dynamic>> temp;
				list<Matrix<double, 7, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				string sen;
				//���½���imu��lidarʱ��ƥ��
				if (!choice)
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
				}
				else
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(70 + (i + 1) * 5 / cccount));
						time_match_m_l_txt(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset, (i + 1));
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
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
			list<Matrix<double, 6, Dynamic >> lidar_final;
			try
			{
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 10);
				lvxReader(fin.lidar_data_file, lidar_final);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]��lidar���ݶ�ȡ����������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 30);
			}
			catch (const char* c)
			{
				strcpy_s(theword, c);
				strcat_s(theword, "\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			int len = fin.imu_data_file.length();
			if (fin.imu_data_file.substr(len - 3, 3) != "txt")
			{
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
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
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
				results = imudetach_2(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 60);
			}
			else
			{
				try {
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ��ʼ��\n");
					(*ob.*settext)(ob, theword);
					imuresult = Read_txt(fin.imu_data_file);
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��imu���ݶ�ȡ������\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, 40);
				}
				catch (const char* c)
				{
					strcpy_s(theword, c);
					strcat_s(theword, "\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]����ʼ�������롤����\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 45);
				results = Aimudetach_2(imuresult, total, rest);
				time(&now);
				strcpy_s(theword, "[");
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				strcat_s(theword, tmp);
				strcat_s(theword, "]������������ɡ�\n");
				(*ob.*settext)(ob, theword);
				imuresult.clear();
				imuresult.shrink_to_fit();
				(*ob.*setvalue)(ob, 60);
			}
			time(&now);
			strcpy_s(theword, "[");
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			strcat_s(theword, tmp);
			strcat_s(theword, "]�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			AGet_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			(*ob.*setvalue)(ob, 65);
			int cccount = results.size();
			for (int i = 0; i < results.size(); i++)
			{
				list<Matrix<double, 6, Dynamic>> temp;
				list<Matrix<double, 6, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}
				string sen;
				//���½���imu��lidarʱ��ƥ��
				if (!choice)
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(65 + (i + 1) * 5 / cccount));
						Atime_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, int(65 + (i + 1) * 15 / cccount));
					if (temp.size() == 0)
					{
						strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						(*ob.*settext)(ob, theword);
						continue;
					}
				}
				else
				{
					try
					{
						time(&now);
						strcpy_s(theword, "[");
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						strcat_s(theword, tmp);
						sen = "]���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						strcat_s(theword, sen.data());
						(*ob.*settext)(ob, theword);
						(*ob.*setvalue)(ob, int(65 + (i + 1) * 5 / cccount));
						Atime_match_m_l_txt(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset, (i + 1));
					}
					catch (const char* c)
					{
						strcpy_s(theword, c);
						strcat_s(theword, "\n");
						(*ob.*settext)(ob, theword);
						return;
					}
					time(&now);
					strcpy_s(theword, "[");
					strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
					strcat_s(theword, tmp);
					strcat_s(theword, "]��ʱ��ƥ����ɡ�\n");
					(*ob.*settext)(ob, theword);
					(*ob.*setvalue)(ob, int(65 + (i + 1) * 15 / cccount));
					if (temp.size() == 0)
					{
						strcpy_s(theword, "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						(*ob.*settext)(ob, theword);
						continue;
					}
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
					AwriteLas_l(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					AwritePcd_l (file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					AwritePly_l(file, temp);
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