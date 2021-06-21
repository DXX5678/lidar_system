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
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ�����ʼ��\n");
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
					(*ob.*setvalue)(ob, ccount * 100 / num);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 0);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePly_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePcd_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ�����ʼ��\n");
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
					(*ob.*setvalue)(ob, ccount * 100 / num);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 0);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePly_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePcd_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try 
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				lidar_final = lvxReader(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ����������\n");
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			time_match_m_m(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_m(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePly_m(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePcd_m(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
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
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ�����ʼ��\n");
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
					(*ob.*setvalue)(ob, ccount * 100 / num);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 0);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			long long num = 0;
			long long current = 0;
			for (int i = 0; i < results.size(); i++)
			{
				num += results[i].cols();
				imufinal.conservativeResize(7, num);
				imufinal.block(0, current, 7, results[i].cols()) = results[i];
				current += results[i].cols();
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePly_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePcd_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ�����ʼ��\n");
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
					(*ob.*setvalue)(ob, ccount * 100 / num);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 0);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			long long num = 0;
			long long current = 0;
			for (int i = 0; i < results.size(); i++)
			{
				num += results[i].cols();
				imufinal.conservativeResize(7, num);
				imufinal.block(0, current, 7, results[i].cols()) = results[i];
				current += results[i].cols();
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_l(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePly_l(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePcd_l(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				lidar_final = lvxReader(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ����������\n");
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			long long num = 0;
			long long current = 0;
			for (int i = 0; i < results.size(); i++)
			{
				num += results[i].cols();
				imufinal.conservativeResize(7, num);
				imufinal.block(0, current, 7, results[i].cols()) = results[i];
				current += results[i].cols();
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼʱ��ƥ�䡣\n");
			(*ob.*settext)(ob, theword);
			time_match_m_m(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
			(*ob.*settext)(ob, theword);
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ���ɵ����ļ�������\n");
			(*ob.*settext)(ob, theword);
			if (fout.points_file_type == 0)
			{
				string file = fout.points_file + ".las";
				writeLas_m(file, lidar_final);
			}
			else if (fout.points_file_type == 1)
			{
				string file = fout.points_file + ".pcd";
				writePly_m(file, lidar_final);
			}
			else if (fout.points_file_type == 2)
			{
				string file = fout.points_file + ".ply";
				writePcd_m(file, lidar_final);
			}
			else
			{
				strcpy_s(theword, "����ļ�����ѡ�����\n");
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�������ļ�������ɡ�\n");
			(*ob.*settext)(ob, theword);
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
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ�����ʼ��\n");
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
					(*ob.*setvalue)(ob, ccount * 100 / num);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 0);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);

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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				string sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcpy_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				if (temp.size() == 0)
				{
					strcpy_s(theword, "ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
					(*ob.*settext)(ob, theword);
					continue;
				}
				//�������ɵ����ļ�
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcpy_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				if (fout.points_file_type == 0)
				{
					string file = fout.points_file + to_string(i + 1) + ".las";
					writeLas_l(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					writePly_l(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					writePcd_l(file, temp);
				}
				else
				{
					strcpy_s(theword, "����ļ�����ѡ�����\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "�������ļ�������ɡ�\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ�����ʼ��\n");
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
					(*ob.*setvalue)(ob, ccount * 100 / num);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݽ���������\n");
				(*ob.*settext)(ob, theword);
				(*ob.*setvalue)(ob, 0);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);

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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				string sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcpy_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				if (temp.size() == 0)
				{
					strcpy_s(theword, "ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
					(*ob.*settext)(ob, theword);
					continue;
				}
				//�������ɵ����ļ�
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcpy_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				if (fout.points_file_type == 0)
				{
					string file = fout.points_file + to_string(i + 1) + ".las";
					writeLas_l(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					writePly_l(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					writePcd_l(file, temp);
				}
				else
				{
					strcpy_s(theword, "����ļ�����ѡ�����\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "�������ļ�������ɡ�\n");
				(*ob.*settext)(ob, theword);
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ������ʼ��\n");
				(*ob.*settext)(ob, theword);
				lidar_final = lvxReader(fin.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��lidar���ݶ�ȡ����������\n");
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ��ʼ��\n");
				(*ob.*settext)(ob, theword);
				imuresult = Read(fin.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��imu���ݶ�ȡ������\n");
				(*ob.*settext)(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				(*ob.*settext)(ob, theword);
				return;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ��ʼ��\n");
			(*ob.*settext)(ob, theword);
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "��imu���ݲ�ֵ������\n");
			(*ob.*settext)(ob, theword);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "����ʼ�������롤����\n");
			(*ob.*settext)(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "������������ɡ�\n");
			(*ob.*settext)(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			(*ob.*settext)(ob, tmp);
			strcpy_s(theword, "�����ð��þ���...\n");
			(*ob.*settext)(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);

			for (int i = 0; i < results.size(); i++)
			{
				MatrixXd temp;
				temp.resize(lidar_final.rows(), lidar_final.cols());
				temp.block(0, 0, lidar_final.rows(), lidar_final.cols()) = lidar_final;
				//���½���imu��lidarʱ��ƥ��
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				string sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcpy_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				time_match_m_m(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "��ʱ��ƥ����ɡ�\n");
				(*ob.*settext)(ob, theword);
				if (temp.cols() == 0)
				{
					strcpy_s(theword, "ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
					(*ob.*settext)(ob, theword);
					continue;
				}
				//�������ɵ����ļ�
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcpy_s(theword, sen.data());
				(*ob.*settext)(ob, theword);
				if (fout.points_file_type == 0)
				{
					string file = fout.points_file + to_string(i + 1) + ".las";
					writeLas_m(file, temp);
				}
				else if (fout.points_file_type == 1)
				{
					string file = fout.points_file + to_string(i + 1) + ".pcd";
					writePly_m(file, temp);
				}
				else if (fout.points_file_type == 2)
				{
					string file = fout.points_file + to_string(i + 1) + ".ply";
					writePcd_m(file, temp);
				}
				else
				{
					strcpy_s(theword, "����ļ�����ѡ�����\n");
					(*ob.*settext)(ob, theword);
					return;
				}
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				(*ob.*settext)(ob, tmp);
				strcpy_s(theword, "�������ļ�������ɡ�\n");
				(*ob.*settext)(ob, theword);
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