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

/*
int main()
{
	time_t now;
	char tmp[16];
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ��ʼ��\n");
		imuresult = Read("sbet_Mission 1.out");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ������\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("��imu���ݲ�ֵ��ʼ��\n");
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
	cout << tmp;
	printf("��imu���ݲ�ֵ������\n");
	ofstream fout("final.txt", ios::out);
	fout.setf(ios::fixed);
	fout.precision(3);
	for (int i = 0; i < imufinal.cols(); i++)
	{
		fout << imufinal(6,i) << endl;
	}
	fout.close();
	return 0;
}
*/


int main()
{
	time_t now;
	char tmp[16];
	int num;
	cout << "�û������߳�����";
	cin >> num;
	//���½���lidar���ݴ���
	list<Matrix<double, 6, Dynamic>> lidar_final;
	list<vector<string>>reading;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݶ�ȡ��ʼ��\n");
		reading = lidar_reading("RS32_2021-03-18_14-11-10-522889.tianyu");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݶ�ȡ������\n");

		
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
		cout << tmp;
		printf("��lidar���ݽ�����ʼ��\n");
		vector<thread>ths;
		vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
		for (int i = 0; i < num; i++)
		{
			ths.push_back(thread(&RSManager::lidar_manage, RSManager(1,32), reading1[i], std::ref(lidar_result[i])));
			reading1[i].clear();
		}
		for (auto iter = ths.begin(); iter != ths.end(); iter++)
		{
			iter->join();
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
		cout << tmp;
		printf("��lidar���ݽ���������\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ��ʼ��\n");
		imuresult = Read("sbet_Mission 1.out");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ������\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp ;
	printf("��imu���ݲ�ֵ��ʼ��\n");
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
	cout << tmp;
	printf("��imu���ݲ�ֵ������\n");
	//���½���imu��lidarʱ��ƥ��
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("�����ð��þ���...\n");
	Get_anzhi_RotationMatrix(0, 0, 0);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("����ʼʱ��ƥ�䡣\n");
	time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("��ʱ��ƥ����ɡ�\n");
	//�������ɵ����ļ�
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("����ʼ���ɵ����ļ�������\n");
	if (0 == 0)
	{
		string file = "test.las";
		writeLas_l(file, lidar_final);
	}
	else if (0 == 1)
	{
		string file = ".pcd";
		writePly_l(file, lidar_final);
	}
	else if (0 == 2)
	{
		string file = ".ply";
		writePcd_l(file, lidar_final);
	}
	else
	{
		printf("����ļ�����ѡ�����\n");
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("�������ļ�������ɡ�\n");
	return 0;
}


/*
int main()
{
	time_t now;
	char tmp[16];
	int num;
	cout << "�û������߳�����";
	cin >> num;
	//���½���lidar���ݴ���
	list<Matrix<double, 6, Dynamic>> lidar_final;
	list<vector<string>>reading;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݶ�ȡ��ʼ��\n");
		reading = lidar_reading("RS32_2021-03-13_10-55-42-633088.tianyu");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݶ�ȡ������\n");


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
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݽ�����ʼ��\n");
		vector<thread>ths;
		vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
		for (int i = 0; i < num; i++)
		{
			ths.push_back(thread(&RSManager::lidar_manage, RSManager(1, 32), reading1[i], std::ref(lidar_result[i])));
			reading1[i].clear();
		}
		for (auto iter = ths.begin(); iter != ths.end(); iter++)
		{
			iter->join();
		}
		ths.clear();

		list<Matrix<double, 6, Dynamic>>::iterator pp;
		for (int i = (num - 1); i >= 0; i--)
		{
			pp = lidar_result[i].begin();
			pp++;
			lidar_final.splice(lidar_final.begin(), lidar_result[i], pp, lidar_result[i].end());
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݽ���������\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ��ʼ��\n");
		imuresult = Read("sbet_Mission_shu.dat");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ������\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("��imu���ݲ�ֵ��ʼ��\n");
	imu_timenew(imuresult);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("��imu���ݲ�ֵ������\n");
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("����ʼ�������롤����\n");
	vector < MatrixXd> results = imudetach(imuresult, total, rest);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("������������ɡ�\n");
	imuresult.clear();
	imuresult.shrink_to_fit();
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("�����ð��þ���...\n");
	Get_anzhi_RotationMatrix(0, 0, 0);

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
		cout << tmp;
		string sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
		printf(sen.data());
		time_match_m_l(results[i], temp, 0.0758, -0.038, -0.002);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��ʱ��ƥ����ɡ�\n");
		if (temp.size() == 0)
		{
			printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
			continue;
		}
		//�������ɵ����ļ�
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
		printf(sen.data());
		if (0 == 0)
		{
			string file = "test" + to_string(i + 1) + ".las";
			writeLas_l(file, temp);
		}
		else if (0 == 1)
		{
			string file = "test" + to_string(i + 1) + ".pcd";
			writePly_l(file, temp);
		}
		else if (0 == 2)
		{
			string file = "test" + to_string(i + 1) + ".ply";
			writePcd_l(file, temp);
		}
		else
		{
			printf("����ļ�����ѡ�����\n");
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("�������ļ�������ɡ�\n");
		temp.clear();
	}
	return 0;
}
*/