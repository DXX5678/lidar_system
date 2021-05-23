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
	void process_n_n(void* ob, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp)
	{
		char theword[100];
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				RSManager RSX1(lp.lidar_work_model, 16);
				RSManager RSX2(lp.lidar_work_model, 16);
				RSManager RSX3(lp.lidar_work_model, 16);
				RSManager RSX4(lp.lidar_work_model, 16);
				strcpy_s(theword, "lidar���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ������\n");
				settext(ob, theword);
				int xxx = reading.size() / 4;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(4);
				for (int i = 0; i < 3; i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[3].splice(reading1[3].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				strcpy_s(theword, "lidar���ݽ�����ʼ��\n");
				settext(ob, theword);
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(4);
				std::thread tw1(&RSManager::lidar_manage, RSX1, reading1[0], std::ref(lidar_result[0]));
				reading1[0].clear();
				std::thread tw2(&RSManager::lidar_manage, RSX2, reading1[1], std::ref(lidar_result[1]));
				reading1[1].clear();
				std::thread tw3(&RSManager::lidar_manage, RSX3, reading1[2], std::ref(lidar_result[2]));
				reading1[2].clear();
				std::thread tw4(&RSManager::lidar_manage, RSX4, reading1[3], std::ref(lidar_result[3]));
				reading1[3].clear();

				tw1.join();
				setvalue(ob, 25);
				tw2.join();
				setvalue(ob, 50);
				tw3.join();
				setvalue(ob, 75);
				tw4.join();
				setvalue(ob, 100);

				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = 3; i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				strcpy_s(theword, "lidar���ݽ���������\n");
				settext(ob, theword);
				setvalue(ob, 0);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
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
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);
			//���½���imu��lidarʱ��ƥ��
			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			strcpy_s(theword, "��ʼʱ��ƥ�䡣\n");
			settext(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
			settext(ob, theword);
			//�������ɵ����ļ�
			strcpy_s(theword, "��ʼ���ɵ����ļ�������\n");
			settext(ob, theword);
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
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "�����ļ�������ɡ�\n");
			settext(ob, theword);
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				RSManager RSX1(lp.lidar_work_model, 32);
				RSManager RSX2(lp.lidar_work_model, 32);
				RSManager RSX3(lp.lidar_work_model, 32);
				RSManager RSX4(lp.lidar_work_model, 32);
				strcpy_s(theword, "lidar���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ������\n");
				settext(ob, theword);
				int xxx = reading.size() / 4;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(4);
				for (int i = 0; i < 3; i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[3].splice(reading1[3].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				strcpy_s(theword, "lidar���ݽ�����ʼ��\n");
				settext(ob, theword);
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(4);
				std::thread tw1(&RSManager::lidar_manage, RSX1, reading1[0], std::ref(lidar_result[0]));
				reading1[0].clear();
				std::thread tw2(&RSManager::lidar_manage, RSX2, reading1[1], std::ref(lidar_result[1]));
				reading1[1].clear();
				std::thread tw3(&RSManager::lidar_manage, RSX3, reading1[2], std::ref(lidar_result[2]));
				reading1[2].clear();
				std::thread tw4(&RSManager::lidar_manage, RSX4, reading1[3], std::ref(lidar_result[3]));
				reading1[3].clear();

				tw1.join();
				setvalue(ob, 25);
				tw2.join();
				setvalue(ob, 50);
				tw3.join();
				setvalue(ob, 75);
				tw4.join();
				setvalue(ob, 100);

				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = 3; i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				strcpy_s(theword, "lidar���ݽ���������\n");
				settext(ob, theword);
				setvalue(ob, 0);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
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
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);
			//���½���imu��lidarʱ��ƥ��
			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			strcpy_s(theword, "��ʼʱ��ƥ�䡣\n");
			settext(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
			settext(ob, theword);
			//�������ɵ����ļ�
			strcpy_s(theword, "��ʼ���ɵ����ļ�������\n");
			settext(ob, theword);
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
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "�����ļ�������ɡ�\n");
			settext(ob, theword);
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try 
			{
				strcpy_s(theword, "lidar���ݶ�ȡ������ʼ��\n");
				settext(ob, theword);
				lidar_final = lvxReader(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ����������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
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
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);
			//���½���imu��lidarʱ��ƥ��
			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			strcpy_s(theword, "��ʼʱ��ƥ�䡣\n");
			settext(ob, theword);
			time_match_m_m(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
			settext(ob, theword);
			//�������ɵ����ļ�
			strcpy_s(theword, "��ʼ���ɵ����ļ�������\n");
			settext(ob, theword);
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
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "�����ļ�������ɡ�\n");
			settext(ob, theword);
		}
		else
		{
			strcpy_s(theword, "�״������쳣��\n");
			settext(ob, theword);
			return;
		}
	}

	void process_y_n(void* ob, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp)
	{
		char theword[100];
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				RSManager RSX1(lp.lidar_work_model, 16);
				RSManager RSX2(lp.lidar_work_model, 16);
				RSManager RSX3(lp.lidar_work_model, 16);
				RSManager RSX4(lp.lidar_work_model, 16);
				strcpy_s(theword, "lidar���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ������\n");
				settext(ob, theword);
				int xxx = reading.size() / 4;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(4);
				for (int i = 0; i < 3; i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[3].splice(reading1[3].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				strcpy_s(theword, "lidar���ݽ�����ʼ��\n");
				settext(ob, theword);
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(4);
				std::thread tw1(&RSManager::lidar_manage, RSX1, reading1[0], std::ref(lidar_result[0]));
				reading1[0].clear();
				std::thread tw2(&RSManager::lidar_manage, RSX2, reading1[1], std::ref(lidar_result[1]));
				reading1[1].clear();
				std::thread tw3(&RSManager::lidar_manage, RSX3, reading1[2], std::ref(lidar_result[2]));
				reading1[2].clear();
				std::thread tw4(&RSManager::lidar_manage, RSX4, reading1[3], std::ref(lidar_result[3]));
				reading1[3].clear();

				tw1.join();
				setvalue(ob, 25);
				tw2.join();
				setvalue(ob, 50);
				tw3.join();
				setvalue(ob, 75);
				tw4.join();
				setvalue(ob, 100);

				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = 3; i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				strcpy_s(theword, "lidar���ݽ���������\n");
				settext(ob, theword);
				setvalue(ob, 0);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
			imu_timenew(imuresult);
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);

			strcpy_s(theword, "��ʼ�������롤����\n");
			settext(ob, theword);
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
			strcpy_s(theword, "����������ɡ�\n");
			settext(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			strcpy_s(theword, "��ʼʱ��ƥ�䡣\n");
			settext(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
			settext(ob, theword);
			//�������ɵ����ļ�
			strcpy_s(theword, "��ʼ���ɵ����ļ�������\n");
			settext(ob, theword);
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
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "�����ļ�������ɡ�\n");
			settext(ob, theword);
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				RSManager RSX1(lp.lidar_work_model, 32);
				RSManager RSX2(lp.lidar_work_model, 32);
				RSManager RSX3(lp.lidar_work_model, 32);
				RSManager RSX4(lp.lidar_work_model, 32);
				strcpy_s(theword, "lidar���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ������\n");
				settext(ob, theword);
				int xxx = reading.size() / 4;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(4);
				for (int i = 0; i < 3; i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[3].splice(reading1[3].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				strcpy_s(theword, "lidar���ݽ�����ʼ��\n");
				settext(ob, theword);
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(4);
				std::thread tw1(&RSManager::lidar_manage, RSX1, reading1[0], std::ref(lidar_result[0]));
				reading1[0].clear();
				std::thread tw2(&RSManager::lidar_manage, RSX2, reading1[1], std::ref(lidar_result[1]));
				reading1[1].clear();
				std::thread tw3(&RSManager::lidar_manage, RSX3, reading1[2], std::ref(lidar_result[2]));
				reading1[2].clear();
				std::thread tw4(&RSManager::lidar_manage, RSX4, reading1[3], std::ref(lidar_result[3]));
				reading1[3].clear();

				tw1.join();
				setvalue(ob, 25);
				tw2.join();
				setvalue(ob, 50);
				tw3.join();
				setvalue(ob, 75);
				tw4.join();
				setvalue(ob, 100);

				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = 3; i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				strcpy_s(theword, "lidar���ݽ���������\n");
				settext(ob, theword);
				setvalue(ob, 0);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
			imu_timenew(imuresult);
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);

			strcpy_s(theword, "��ʼ�������롤����\n");
			settext(ob, theword);
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
			strcpy_s(theword, "����������ɡ�\n");
			settext(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			strcpy_s(theword, "��ʼʱ��ƥ�䡣\n");
			settext(ob, theword);
			time_match_m_l(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
			settext(ob, theword);
			//�������ɵ����ļ�
			strcpy_s(theword, "��ʼ���ɵ����ļ�������\n");
			settext(ob, theword);
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
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "�����ļ�������ɡ�\n");
			settext(ob, theword);
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try
			{
				strcpy_s(theword, "lidar���ݶ�ȡ������ʼ��\n");
				settext(ob, theword);
				lidar_final = lvxReader(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ����������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
			imu_timenew(imuresult);
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);

			strcpy_s(theword, "��ʼ�������롤����\n");
			settext(ob, theword);
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
			strcpy_s(theword, "����������ɡ�\n");
			settext(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);
			strcpy_s(theword, "��ʼʱ��ƥ�䡣\n");
			settext(ob, theword);
			time_match_m_m(imufinal, lidar_final, cp.x_offset, cp.y_offset, cp.z_offset);
			strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
			settext(ob, theword);
			//�������ɵ����ļ�
			strcpy_s(theword, "��ʼ���ɵ����ļ�������\n");
			settext(ob, theword);
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
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "�����ļ�������ɡ�\n");
			settext(ob, theword);
		}
		else
		{
			strcpy_s(theword, "�״������쳣��\n");
			settext(ob, theword);
			return;
		}
	}

	void process_y_y(void* ob, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input fin, file_output fout, lidar_parameter lp, coordinate_parameter cp)
	{
		char theword[100];
		if (lp.lidar_type == 0)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				RSManager RSX1(lp.lidar_work_model, 16);
				RSManager RSX2(lp.lidar_work_model, 16);
				RSManager RSX3(lp.lidar_work_model, 16);
				RSManager RSX4(lp.lidar_work_model, 16);
				strcpy_s(theword, "lidar���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ������\n");
				settext(ob, theword);
				int xxx = reading.size() / 4;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(4);
				for (int i = 0; i < 3; i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[3].splice(reading1[3].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				strcpy_s(theword, "lidar���ݽ�����ʼ��\n");
				settext(ob, theword);
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(4);
				std::thread tw1(&RSManager::lidar_manage, RSX1, reading1[0], std::ref(lidar_result[0]));
				reading1[0].clear();
				std::thread tw2(&RSManager::lidar_manage, RSX2, reading1[1], std::ref(lidar_result[1]));
				reading1[1].clear();
				std::thread tw3(&RSManager::lidar_manage, RSX3, reading1[2], std::ref(lidar_result[2]));
				reading1[2].clear();
				std::thread tw4(&RSManager::lidar_manage, RSX4, reading1[3], std::ref(lidar_result[3]));
				reading1[3].clear();

				tw1.join();
				setvalue(ob, 25);
				tw2.join();
				setvalue(ob, 50);
				tw3.join();
				setvalue(ob, 75);
				tw4.join();
				setvalue(ob, 100);

				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = 3; i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				strcpy_s(theword, "lidar���ݽ���������\n");
				settext(ob, theword);
				setvalue(ob, 0);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
			imu_timenew(imuresult);
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);

			strcpy_s(theword, "��ʼ�������롤����\n");
			settext(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			strcpy_s(theword, "����������ɡ�\n");
			settext(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();

			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
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
				string sen = "�Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcpy_s(theword, sen.data());
				settext(ob, theword);
				time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
				settext(ob, theword);
				//�������ɵ����ļ�
				sen = "�Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcpy_s(theword, sen.data());
				settext(ob, theword);
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
					settext(ob, theword);
					return;
				}
				strcpy_s(theword, "�����ļ�������ɡ�\n");
				settext(ob, theword);
				temp.clear();
			}
		}
		else if (lp.lidar_type == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				RSManager RSX1(lp.lidar_work_model, 32);
				RSManager RSX2(lp.lidar_work_model, 32);
				RSManager RSX3(lp.lidar_work_model, 32);
				RSManager RSX4(lp.lidar_work_model, 32);
				strcpy_s(theword, "lidar���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				reading = lidar_reading(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ������\n");
				settext(ob, theword);
				int xxx = reading.size() / 4;
				list<vector<string>>::iterator first1 = reading.begin();
				list<vector<string>>::iterator last;
				vector<list<vector<string>>>reading1(4);
				for (int i = 0; i < 3; i++)
				{
					last = reading.begin();
					for (int j = 0; j < xxx; j++)
					{
						last++;
					}
					reading1[i].splice(reading1[i].begin(), reading, reading.begin(), last);
				}
				reading1[3].splice(reading1[3].begin(), reading, reading.begin(), reading.end());
				reading.clear();
				strcpy_s(theword, "lidar���ݽ�����ʼ��\n");
				settext(ob, theword);
				vector<list<Matrix<double, 6, Dynamic>>>lidar_result(4);
				std::thread tw1(&RSManager::lidar_manage, RSX1, reading1[0], std::ref(lidar_result[0]));
				reading1[0].clear();
				std::thread tw2(&RSManager::lidar_manage, RSX2, reading1[1], std::ref(lidar_result[1]));
				reading1[1].clear();
				std::thread tw3(&RSManager::lidar_manage, RSX3, reading1[2], std::ref(lidar_result[2]));
				reading1[2].clear();
				std::thread tw4(&RSManager::lidar_manage, RSX4, reading1[3], std::ref(lidar_result[3]));
				reading1[3].clear();

				tw1.join();
				setvalue(ob, 25);
				tw2.join();
				setvalue(ob, 50);
				tw3.join();
				setvalue(ob, 75);
				tw4.join();
				setvalue(ob, 100);

				list<Matrix<double, 6, Dynamic>>::iterator pp;
				for (int i = 3; i >= 0; i--)
				{
					pp = lidar_result[i].begin();
					pp++;
					lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
				}
				strcpy_s(theword, "lidar���ݽ���������\n");
				settext(ob, theword);
				setvalue(ob, 0);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
			imu_timenew(imuresult);
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);

			strcpy_s(theword, "��ʼ�������롤����\n");
			settext(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			strcpy_s(theword, "����������ɡ�\n");
			settext(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();

			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
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
				string sen = "�Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcpy_s(theword, sen.data());
				settext(ob, theword);
				time_match_m_l(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
				settext(ob, theword);
				//�������ɵ����ļ�
				sen = "�Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcpy_s(theword, sen.data());
				settext(ob, theword);
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
					settext(ob, theword);
					return;
				}
				strcpy_s(theword, "�����ļ�������ɡ�\n");
				settext(ob, theword);
				temp.clear();
			}
		}
		else if (lp.lidar_type == 2)
		{
			//���½���lidar���ݴ���
			MatrixXd lidar_final;
			try
			{
				strcpy_s(theword, "lidar���ݶ�ȡ������ʼ��\n");
				settext(ob, theword);
				lidar_final = lvxReader(fin.lidar_data_file);
				strcpy_s(theword, "lidar���ݶ�ȡ����������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				strcpy_s(theword, "imu���ݶ�ȡ��ʼ��\n");
				settext(ob, theword);
				imuresult = Read(fin.imu_data_file);
				strcpy_s(theword, "imu���ݶ�ȡ������\n");
				settext(ob, theword);
			}
			catch (string mes)
			{
				mes += "\n";
				strcpy_s(theword, mes.data());
				settext(ob, theword);
				return;
			}
			strcpy_s(theword, "imu���ݲ�ֵ��ʼ��\n");
			settext(ob, theword);
			imu_timenew(imuresult);
			strcpy_s(theword, "imu���ݲ�ֵ������\n");
			settext(ob, theword);

			strcpy_s(theword, "��ʼ�������롤����\n");
			settext(ob, theword);
			vector < MatrixXd> results = imudetach(imuresult, total, rest);
			strcpy_s(theword, "����������ɡ�\n");
			settext(ob, theword);
			imuresult.clear();
			imuresult.shrink_to_fit();

			strcpy_s(theword, "���ð��þ���...\n");
			settext(ob, theword);
			Get_anzhi_RotationMatrix(cp.alpha, cp.beta, cp.gamma);

			for (int i = 0; i < results.size(); i++)
			{
				MatrixXd temp;
				temp.resize(lidar_final.rows(), lidar_final.cols());
				temp.block(0, 0, lidar_final.rows(), lidar_final.cols()) = lidar_final;
				//���½���imu��lidarʱ��ƥ��
				string sen = "�Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				strcpy_s(theword, sen.data());
				settext(ob, theword);
				time_match_m_m(results[i], temp, cp.x_offset, cp.y_offset, cp.z_offset);
				strcpy_s(theword, "ʱ��ƥ����ɡ�\n");
				settext(ob, theword);
				//�������ɵ����ļ�
				sen = "�Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				strcpy_s(theword, sen.data());
				settext(ob, theword);
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
					settext(ob, theword);
					return;
				}
				strcpy_s(theword, "�����ļ�������ɡ�\n");
				settext(ob, theword);
			}
		}
		else
		{
			strcpy_s(theword, "�״������쳣��\n");
			settext(ob, theword);
			return;
		}
	}
}