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
#include "IMUanalyze.h"


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

/*
int main()
{
	time_t now;
	char tmp[16];
	//���½���lidar���ݴ���
	MatrixXd lidar_final;
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ������ʼ��\n";
		lidar_final = lvxReader("2021-07-19_20-53-12.lvx");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ����������\n";
		writeLas_m("lvx.las", lidar_final);
	}
	catch (string mes)
	{
		mes += "\n";
		cout << mes.data();
		return 0;
	}
	return 0;
}
*/

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
		reading = lidar_reading("RS32_2021-07-26_16-15-44-856204.tianyu");
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
		//printf("lidar�������һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", lidar_final.front()(0, 0), lidar_final.front()(1, 0), lidar_final.front()(2, 0), lidar_final.front()(3, 0), lidar_final.front()(4, 0), lidar_final.front()(5, 0));
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
		imuresult = Read("sbet_Mission 12.out");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ������\n");
		//printf("imu��ȡ���һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", imuresult[0](0, 0), imuresult[0](1, 0), imuresult[0](2, 0), imuresult[0](3, 0), imuresult[0](4, 0), imuresult[0](5, 0), imuresult[0](6, 0));
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
	//printf("imu��ֵ���һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", imufinal(0, 0), imufinal(1, 0), imufinal(2, 0), imufinal(3, 0), imufinal(4, 0), imufinal(5, 0), imufinal(6, 0));
	//���½���imu��lidarʱ��ƥ��
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("�����ð��þ���...\n");
	Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("����ʼʱ��ƥ�䡣\n");
	time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("��ʱ��ƥ����ɡ�\n");
	//printf("lidarƥ����һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", lidar_final.front()(0, 0), lidar_final.front()(1, 0), lidar_final.front()(2, 0), lidar_final.front()(3, 0), lidar_final.front()(4, 0), lidar_final.front()(5, 0));
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
		string file = "test.pcd";
		writePcd_l(file, lidar_final);
	}
	else if (0 == 2)
	{
		string file = "test.ply";
		writePly_l(file, lidar_final); 
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
*/

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
		cout<<"��lidar���ݶ�ȡ��ʼ��\n";
		reading = lidar_reading("RS32_2021-07-26_16-15-44-856204.tianyu");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout<<"��lidar���ݶ�ȡ������\n";
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
		cout << "��lidar���ݽ�����ʼ��\n";
		vector<thread>ths;
		vector<list<Matrix<double, 6, Dynamic>>>lidar_result(num);
		for (int i = 0; i < num; i++)
		{
			ths.push_back(thread(&RSManager::lidar_manage, RSManager(1, 32), reading1[i], std::ref(lidar_result[i])));
			reading1[i].clear();
		}
		int ccount = 1;
		for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
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
		cout << "��lidar���ݽ���������\n";
	}
	catch (string mes)
	{
		mes += "\n";
		cout << mes;
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݶ�ȡ��ʼ��\n";
		imuresult = Read("sbet_Mission 1.out");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݶ�ȡ������\n";
	}
	catch (string mes)
	{
		mes += "\n";
		cout << mes;
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "��imu���ݲ�ֵ��ʼ��\n";
	imu_timenew(imuresult);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "��imu���ݲ�ֵ������\n";
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "����ʼ�������롤����\n";
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
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "������������ɡ�\n";
	imuresult.clear();
	imuresult.shrink_to_fit();
	results.clear();
	results.shrink_to_fit();

	//���½���imu��lidarʱ��ƥ��
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�����ð��þ���...\n";
	Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "����ʼʱ��ƥ�䡣\n";
	time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "��ʱ��ƥ����ɡ�\n";
	//�������ɵ����ļ�
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "����ʼ���ɵ����ļ�������\n";
	if (0 == 0)
	{
		string file =  "tests.las";
		writeLas_l(file, lidar_final);
	}
	else if (0 == 1)
	{
		string file = "tests.pcd";
		writePcd_l(file, lidar_final);
	}
	else if (0 == 2)
	{
		string file = "tests.ply";
		writePly_l(file, lidar_final);
	}
	else
	{
		cout << "����ļ�����ѡ�����\n";
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�������ļ�������ɡ�\n";
	return 0;
}
*/

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
		reading = lidar_reading("RS32_2021-07-26_16-15-44-856204.tianyu");
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
	printf("������������ɣ���%d��������\n",int(results.size()));
	imuresult.clear();
	imuresult.shrink_to_fit();
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("�����ð��þ���...\n");
	Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);

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
			writePcd_l(file, temp);
		}
		else if (0 == 2)
		{
			string file = "test" + to_string(i + 1) + ".ply";
			writePly_l(file, temp);
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

/*
int main()
{
	time_t now;
	char tmp[16];
	//���½���lidar���ݴ���
	MatrixXd lidar_final;
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ������ʼ��\n";
		lidar_final = lvxReader("2021-07-26_18-02-36.lid");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ����������\n";
	}
	catch (const char* c)
	{
		cout << c << endl;
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	string imu_data_file = "3.txt";
	int len = imu_data_file.length();
	if (imu_data_file.substr(len - 3, 3) != "txt")
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ��ʼ��\n";
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
		cout << "��imu���ݲ�ֵ������\n";
	}
	else
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read_txt(imu_data_file);
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
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
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݶ�ȡ������\n";
	}
	//���½���imu��lidarʱ��ƥ��
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�����ð��þ���...\n";
	AGet_anzhi_RotationMatrix(-0.01136, 0.025027, 0.01549);
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼʱ��ƥ�䡣\n";
		//Atime_match_m_m(imufinal, lidar_final, 0.0758, -0.038, -0.002);
		Atime_match_m_m_txt(imufinal, lidar_final, 0.0758, -0.038, -0.002);

	}
	catch (const char* c)
	{
		cout << c << endl;
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "��ʱ��ƥ����ɡ�\n";
	//�������ɵ����ļ�
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "����ʼ���ɵ����ļ�������\n";
	int points_file_type = 0;
	if (points_file_type == 0)
	{
		string file = "lvx.las";
		writeLas_m(file, lidar_final);
	}
	else if (points_file_type == 1)
	{
		string file = "lvx.pcd";
		writePcd_m(file, lidar_final);
	}
	else if (points_file_type == 2)
	{
		string file = "lvx.ply";
		writePly_m(file, lidar_final);
	}
	else
	{
		cout << "����ļ�����ѡ�����\n";
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�������ļ�������ɡ�\n";
	return 0;
}
*/

/*
int main()
{
	time_t now;
	char tmp[16];
	//���½���lidar���ݴ���
	MatrixXd lidar_final;
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ������ʼ��\n";
		lidar_final = lvxReader("2021-07-19_20-53-12.lvx");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ����������\n";
	}
	catch (const char* c)
	{
		cout << c << endl;
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	string imu_data_file = "3.txt";
	int len = imu_data_file.length();
	if (imu_data_file.substr(len - 3, 3) != "txt")
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "]��imu���ݲ�ֵ��ʼ��\n";
		imu_timenew(imuresult);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ������\n";
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
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
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɡ�\n";
		imuresult.clear();
		imuresult.shrink_to_fit();
		results.clear();
		results.shrink_to_fit();
	}
	else
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read_txt(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
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
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɡ�\n";
		imuresult.clear();
		imuresult.shrink_to_fit();
		results.clear();
		results.shrink_to_fit();
	}
	//���½���imu��lidarʱ��ƥ��
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�����ð��þ���...\n";
	AGet_anzhi_RotationMatrix(-0.01136, 0.025027, 0.01549);
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼʱ��ƥ�䡣\n";
		Atime_match_m_m(imufinal, lidar_final, 0.0758, -0.038, -0.002);
	}
	catch (const char* c)
	{
		cout << c << endl;
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "��ʱ��ƥ����ɡ�\n";
	//�������ɵ����ļ�
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "����ʼ���ɵ����ļ�������\n";
	int points_file_type = 0;
	if (points_file_type == 0)
	{
		string file = "lvx.las";
		writeLas_m(file, lidar_final);
	}
	else if (points_file_type == 1)
	{
		string file = "lvx.pcd";
		writePcd_m(file, lidar_final);
	}
	else if (points_file_type == 2)
	{
		string file = "lvx.ply";
		writePly_m(file, lidar_final);
	}
	else
	{
		cout << "����ļ�����ѡ�����\n";
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�������ļ�������ɡ�\n";
	return 0;
}
*/

/*
int main()
{
	time_t now;
	char tmp[16];
	//���½���lidar���ݴ���
	MatrixXd lidar_final;
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ������ʼ��\n";
		lidar_final = lvxReader("2021-07-26_18-02-36.lid");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��lidar���ݶ�ȡ����������\n";
	}
	catch (const char* c)
	{
		cout << c << endl;
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	vector < MatrixXd> results;
	string imu_data_file = "a1-40.txt";
	int len = imu_data_file.length();
	if (imu_data_file.substr(len - 3, 3) != "txt")
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ��ʼ��\n";
		imu_timenew(imuresult);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ������\n";
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
		results = Aimudetach(imuresult, total, rest);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɣ���" << results.size() << "������" << endl;
		imuresult.clear();
		imuresult.shrink_to_fit();
	}
	else
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read_txt(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
		results = Aimudetach(imuresult, total, rest);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɣ���" << results.size() << "������" << endl;
		imuresult.clear();
		imuresult.shrink_to_fit();
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "�����ð��þ���...\n";
	AGet_anzhi_RotationMatrix(-0.01136, 0.025027, 0.01549);
	int cccount = results.size();
	for (int i = 0; i < results.size(); i++)
	{
		MatrixXd temp;
		temp.resize(lidar_final.rows(), lidar_final.cols());
		temp.block(0, 0, lidar_final.rows(), lidar_final.cols()) = lidar_final;
		string sen;
		//���½���imu��lidarʱ��ƥ��
		try
		{
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
			cout << sen;
			Atime_match_m_m(results[i], temp, 0.0758, -0.038, -0.002);
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��ʱ��ƥ����ɡ�\n";
		if (temp.cols() == 0)
		{
			cout << "         ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n";
			continue;
		}
		//�������ɵ����ļ�
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
		cout << sen;
		int points_file_type = 0;
		if (points_file_type == 0)
		{
			string file = "lvx" + to_string(i + 1) + ".las";
			writeLas_m(file, temp);
		}
		else if (points_file_type == 1)
		{
			string file = "lvx" + to_string(i + 1) + ".pcd";
			writePcd_m(file, temp);
		}
		else if (points_file_type == 2)
		{
			string file = "lvx" + to_string(i + 1) + ".ply";
			writePly_m(file, temp);
		}
		else
		{
			cout << "����ļ�����ѡ�����\n";
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "�������ļ�������ɡ�\n";
	}
	return 0;
}
*/

/*
int main()
{
	time_t now;
	char tmp[16];
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	vector < MatrixXd> results;
	string imu_data_file = "a1-40.txt";
	int len = imu_data_file.length();
	if (imu_data_file.substr(len - 3, 3) != "txt")
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ��ʼ��\n";
		imu_timenew(imuresult);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ������\n";
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
		results = Aimudetach(imuresult, total, rest);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɣ���" << results.size() << "������" << endl;
		imuresult.clear();
		imuresult.shrink_to_fit();
	}
	else
	{
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read_txt(imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (const char* c)
		{
			cout << c << endl;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
		results = Aimudetach(imuresult, total, rest);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɣ���" << results.size() << "������" << endl;
		imuresult.clear();
		imuresult.shrink_to_fit();
	}
	return 0;
}*/

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
	printf("������������ɣ���%d��������\n", int(results.size()));
	imuresult.clear();
	imuresult.shrink_to_fit();
	return 0;
}*/

/*int main()
{
	time_t now;
	char tmp[16];
	int num;
	cout << "�û������߳�����";
	cin >> num;
	int choice;
	cout << "ѡ��ģʽ(�����뺽������1 ȥ���յ㲻�ֿ�����las�ļ�����2 ȥ���յ�ֿ�����las�ļ�����3 ���뺽���ֿ�����las�ļ�����4)��";
	cin >> choice;
	int choice2;
	cout << "ѡ���Ƿ���������txt�ļ�(������0 ������1)��";
	cin >> choice2;
	Interface::file_input in;
	cout << "����lidar�����ļ���ַ��";
	cin >> in.lidar_data_file;
	cout << "����imu�����ļ���ַ��";
	cin >> in.imu_data_file;
	if (choice == 1)
	{
		//���½���lidar���ݴ���
		list<Matrix<double, 7, Dynamic>> lidar_final;
		list<vector<string>>reading;
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��lidar���ݶ�ȡ��ʼ��\n");
			reading = lidar_reading(in.lidar_data_file);
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
			vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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

			list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			//printf("lidar�������һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", lidar_final.front()(0, 0), lidar_final.front()(1, 0), lidar_final.front()(2, 0), lidar_final.front()(3, 0), lidar_final.front()(4, 0), lidar_final.front()(5, 0));
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
			imuresult = Read(in.imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��imu���ݶ�ȡ������\n");
			//printf("imu��ȡ���һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", imuresult[0](0, 0), imuresult[0](1, 0), imuresult[0](2, 0), imuresult[0](3, 0), imuresult[0](4, 0), imuresult[0](5, 0), imuresult[0](6, 0));
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
		//printf("imu��ֵ���һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", imufinal(0, 0), imufinal(1, 0), imufinal(2, 0), imufinal(3, 0), imufinal(4, 0), imufinal(5, 0), imufinal(6, 0));
		//���½���imu��lidarʱ��ƥ��
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("�����ð��þ���...\n");
		Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);

		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		if (choice2 == 0)
		{
			printf("����ʼʱ��ƥ��ͬʱ����txt�ļ���\n");
			time_match_m_l_txt(imufinal, lidar_final, 0.0758, -0.038, -0.002);
		}
		else
		{
			printf("����ʼʱ��ƥ�䡣\n");
			time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��ʱ��ƥ����ɡ�\n");
		//printf("lidarƥ����һ������Ϊ��%18.9f, %18.9f, %18.9f, %18.9f, %18.9f, %18.9f\n", lidar_final.front()(0, 0), lidar_final.front()(1, 0), lidar_final.front()(2, 0), lidar_final.front()(3, 0), lidar_final.front()(4, 0), lidar_final.front()(5, 0));
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
			string file = "test.pcd";
			writePcd_l(file, lidar_final);
		}
		else if (0 == 2)
		{
			string file = "test.ply";
			writePly_l(file, lidar_final);
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
	}
	else if (choice == 2)
	{
		//���½���lidar���ݴ���
		list<Matrix<double, 7, Dynamic>> lidar_final;
		list<vector<string>>reading;
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��lidar���ݶ�ȡ��ʼ��\n";
			reading = lidar_reading(in.lidar_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��lidar���ݶ�ȡ������\n";
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
			cout << "��lidar���ݽ�����ʼ��\n";
			vector<thread>ths;
			vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
			for (int i = 0; i < num; i++)
			{
				ths.push_back(thread(&RSManager::lidar_manage, RSManager(1, 32), reading1[i], std::ref(lidar_result[i])));
				reading1[i].clear();
			}
			int ccount = 1;
			for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
			{
				iter->join();
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��lidar���ݽ���������\n";
		}
		catch (string mes)
		{
			mes += "\n";
			cout << mes;
			return 0;
		}
		//���½���imu���ݴ���
		vector < Matrix<double, 7, 2000>> imuresult;
		MatrixXd imufinal;
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ��ʼ��\n";
			imuresult = Read(in.imu_data_file);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݶ�ȡ������\n";
		}
		catch (string mes)
		{
			mes += "\n";
			cout << mes;
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ��ʼ��\n";
		imu_timenew(imuresult);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��imu���ݲ�ֵ������\n";
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ�������롤����\n";
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
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "������������ɡ�\n";
		imuresult.clear();
		imuresult.shrink_to_fit();
		results.clear();
		results.shrink_to_fit();

		//���½���imu��lidarʱ��ƥ��
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "�����ð��þ���...\n";
		Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		if (choice2 == 0)
		{
			printf("����ʼʱ��ƥ��ͬʱ����txt�ļ���\n");
			time_match_m_l_txt(imufinal, lidar_final, 0.0758, -0.038, -0.002);
		}
		else
		{
			printf("����ʼʱ��ƥ�䡣\n");
			time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "��ʱ��ƥ����ɡ�\n";
		//�������ɵ����ļ�
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "����ʼ���ɵ����ļ�������\n";
		if (0 == 0)
		{
			string file = "tests.las";
			writeLas_l(file, lidar_final);
		}
		else if (0 == 1)
		{
			string file = "tests.pcd";
			writePcd_l(file, lidar_final);
		}
		else if (0 == 2)
		{
			string file = "tests.ply";
			writePly_l(file, lidar_final);
		}
		else
		{
			cout << "����ļ�����ѡ�����\n";
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "�������ļ�������ɡ�\n";
	}
	else if (choice == 3)
	{
		//���½���lidar���ݴ���
		list<Matrix<double, 7, Dynamic>> lidar_final;
		list<vector<string>>reading;
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��lidar���ݶ�ȡ��ʼ��\n");
			reading = lidar_reading(in.lidar_data_file);
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
			vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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

			list<Matrix<double, 7, Dynamic>>::iterator pp;
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
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��imu���ݶ�ȡ��ʼ��\n");
			imuresult = Read(in.imu_data_file);
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
		printf("������������ɣ���%d��������\n", int(results.size()));
		imuresult.clear();
		imuresult.shrink_to_fit();
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("�����ð��þ���...\n");
		Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);

		for (int i = 0; i < results.size(); i++)
		{
			list<Matrix<double, 7, Dynamic>> temp;
			list<Matrix<double, 7, Dynamic>>::iterator ap;
			for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
			{
				temp.push_back(*ap);
			}
			
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			string sen;
			if (choice2 == 0)
			{
				sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ��ͬʱ����txt�ļ���\n";
				printf(sen.data());
				time_match_m_l_txt(results[i], temp, 0.0758, -0.038, -0.002, (i + 1));
			}
			else
			{
				sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				printf(sen.data());
				time_match_m_l(results[i], temp, 0.0758, -0.038, -0.002);
			}
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
				writePcd_l(file, temp);
			}
			else if (0 == 2)
			{
				string file = "test" + to_string(i + 1) + ".ply";
				writePly_l(file, temp);
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
	}
	else if (choice == 4)
	{
		//���½���lidar���ݴ���
		list<Matrix<double, 7, Dynamic>> lidar_final;
		list<vector<string>>reading;
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��lidar���ݶ�ȡ��ʼ��\n");
			reading = lidar_reading(in.lidar_data_file);
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
			vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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

			list<Matrix<double, 7, Dynamic>>::iterator pp;
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
		try {
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��imu���ݶ�ȡ��ʼ��\n");
			imuresult = Read(in.imu_data_file);
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
		vector < MatrixXd> results = imudetach_2(imuresult, total, rest);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("������������ɣ���%d��������\n", int(results.size()));
		imuresult.clear();
		imuresult.shrink_to_fit();
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("�����ð��þ���...\n");
		Get_anzhi_RotationMatrix(-0.0010807, 0.038508849, -0.009199);

		for (int i = 0; i < results.size(); i++)
		{
			list<Matrix<double, 7, Dynamic>> temp;
			list<Matrix<double, 7, Dynamic>>::iterator ap;
			for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
			{
				temp.push_back(*ap);
			}

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			string sen;
			if (choice2 == 0)
			{
				sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ��ͬʱ����txt�ļ���\n";
				printf(sen.data());
				time_match_m_l_txt(results[i], temp, 0.0758, -0.038, -0.002, (i + 1));
			}
			else
			{
				sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				printf(sen.data());
				time_match_m_l(results[i], temp, 0.0758, -0.038, -0.002);
			}
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
				writePcd_l(file, temp);
			}
			else if (0 == 2)
			{
				string file = "test" + to_string(i + 1) + ".ply";
				writePly_l(file, temp);
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
	}
	else
	{
		cout << "ģʽ��ƥ��" << endl;
		return 0;
	}
	return 0;
}*/

//�ֺ�����ͼ
/*int main()
{
	time_t now;
	char tmp[16];
	Interface::file_input in;
	cout << "����imu�����ļ���ַ��";
	cin >> in.imu_data_file;
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ��ʼ��\n");
		imuresult = Read_2(in.imu_data_file);
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
	cout << "����ʼ�������롤����\n";
	vector < MatrixXd> results = imudetach_2(imuresult, total, rest);

	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "������������ɡ�\n";
	cout << results.size() << endl;
	imuresult.clear();
	imuresult.shrink_to_fit();
	//results.clear();
	//results.shrink_to_fit();
	for (int i = 0; i < results.size(); i++)
	{
		//�������ɵ����ļ�
		if (0 == 0)
		{
			string file = "test" + to_string(i + 1) + ".las";
			write_imu(file, results[i]);
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
	}
	return 0;
}*/

//������
int main()
{
	time_t now;
	char tmp[16];
	int model;
	cout << "�û�ѡ���״�����(RS32����0 Avia����1)��";
	cin >> model;
	int choice;
	cout << "ѡ��ģʽ(�����뺽������1 ȥ���յ㲻�ֿ�����las�ļ�����2 ȥ���յ�ֿ�����las�ļ�����3 ���뺽���ֿ�����las�ļ�����4)��";
	cin >> choice;
	int choice2;
	cout << "ѡ���Ƿ���������txt�ļ�(������0 ������1)��";
	cin >> choice2;
	int choice3;
	cout << "ѡ�����ɵ����ļ���ʽ(las����0 pcd����1 ply����2)��";
	cin >> choice3;
	double a, b, g, xx, yy, zz;
	cout << "���ð��ý�(Alpha Beta Gamma):";
	cin >> a >> b >> g;
	cout << "����ƽ����(x���� y���� z����):";
	cin >> xx >> yy >> zz;
	Interface::file_input in;
	cout << "����lidar�����ļ���ַ��";
	cin >> in.lidar_data_file;
	cout << "����imu�����ļ���ַ��";
	cin >> in.imu_data_file;
	if (model == 0)
	{
		int num;
		cout << "�û������߳�����";
		cin >> num;
		if (choice == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ��ʼ��\n");
				reading = lidar_reading(in.lidar_data_file);
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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

				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
				imuresult = Read(in.imu_data_file);
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
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			Get_anzhi_RotationMatrix(a, b, g);

			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			if (choice2 == 0)
			{
				printf("����ʼʱ��ƥ��ͬʱ����txt�ļ���\n");
				time_match_m_l_txt(imufinal, lidar_final, xx, yy, zz);
			}
			else
			{
				printf("����ʼʱ��ƥ�䡣\n");
				time_match_m_l(imufinal, lidar_final, xx, yy, zz);
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��ʱ��ƥ����ɡ�\n");
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("����ʼ���ɵ����ļ�������\n");
			if (choice3 == 0)
			{
				string file = "test.las";
				writeLas_l(file, lidar_final);
			}
			else if (choice3 == 1)
			{
				string file = "test.pcd";
				writePcd_l(file, lidar_final);
			}
			else if (choice3 == 2)
			{
				string file = "test.ply";
				writePly_l(file, lidar_final);
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
		}
		else if (choice == 2)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				cout << "��lidar���ݶ�ȡ��ʼ��\n";
				reading = lidar_reading(in.lidar_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				cout << "��lidar���ݶ�ȡ������\n";
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
				cout << "��lidar���ݽ�����ʼ��\n";
				vector<thread>ths;
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
				for (int i = 0; i < num; i++)
				{
					ths.push_back(thread(&RSManager::lidar_manage, RSManager(1, 32), reading1[i], std::ref(lidar_result[i])));
					reading1[i].clear();
				}
				int ccount = 1;
				for (auto iter = ths.begin(); iter != ths.end(); iter++, ccount++)
				{
					iter->join();
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
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				cout << "��lidar���ݽ���������\n";
			}
			catch (string mes)
			{
				mes += "\n";
				cout << mes;
				return 0;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			MatrixXd imufinal;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				cout << "��imu���ݶ�ȡ��ʼ��\n";
				imuresult = Read(in.imu_data_file);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				cout << "��imu���ݶ�ȡ������\n";
			}
			catch (string mes)
			{
				mes += "\n";
				cout << mes;
				return 0;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݲ�ֵ��ʼ��\n";
			imu_timenew(imuresult);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��imu���ݲ�ֵ������\n";
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "����ʼ�������롤����\n";
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "������������ɡ�\n";
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();

			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "�����ð��þ���...\n";
			Get_anzhi_RotationMatrix(a, b, g);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			if (choice2 == 0)
			{
				printf("����ʼʱ��ƥ��ͬʱ����txt�ļ���\n");
				time_match_m_l_txt(imufinal, lidar_final, xx, yy, zz);
			}
			else
			{
				printf("����ʼʱ��ƥ�䡣\n");
				time_match_m_l(imufinal, lidar_final, xx, yy, zz);
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "��ʱ��ƥ����ɡ�\n";
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "����ʼ���ɵ����ļ�������\n";
			if (choice3 == 0)
			{
				string file = "tests.las";
				writeLas_l(file, lidar_final);
			}
			else if (choice3 == 1)
			{
				string file = "tests.pcd";
				writePcd_l(file, lidar_final);
			}
			else if (choice3 == 2)
			{
				string file = "tests.ply";
				writePly_l(file, lidar_final);
			}
			else
			{
				cout << "����ļ�����ѡ�����\n";
				return 0;
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			cout << "�������ļ�������ɡ�\n";
		}
		else if (choice == 3)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ��ʼ��\n");
				reading = lidar_reading(in.lidar_data_file);
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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

				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��imu���ݶ�ȡ��ʼ��\n");
				imuresult = Read(in.imu_data_file);
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
			printf("������������ɣ���%d��������\n", int(results.size()));
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			Get_anzhi_RotationMatrix(a, b, g);

			for (int i = 0; i < results.size(); i++)
			{
				list<Matrix<double, 7, Dynamic>> temp;
				list<Matrix<double, 7, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}

				//���½���imu��lidarʱ��ƥ��
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				string sen;
				if (choice2 == 0)
				{
					sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ��ͬʱ����txt�ļ���\n";
					printf(sen.data());
					time_match_m_l_txt(results[i], temp, xx, yy, zz, (i + 1));
				}
				else
				{
					sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
					printf(sen.data());
					time_match_m_l(results[i], temp, xx, yy, zz);
				}
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
				if (choice3 == 0)
				{
					string file = "test" + to_string(i + 1) + ".las";
					writeLas_l(file, temp);
				}
				else if (choice3 == 1)
				{
					string file = "test" + to_string(i + 1) + ".pcd";
					writePcd_l(file, temp);
				}
				else if (choice3 == 2)
				{
					string file = "test" + to_string(i + 1) + ".ply";
					writePly_l(file, temp);
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
		}
		else if (choice == 4)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 7, Dynamic>> lidar_final;
			list<vector<string>>reading;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ��ʼ��\n");
				reading = lidar_reading(in.lidar_data_file);
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
				vector<list<Matrix<double, 7, Dynamic>>>lidar_result(num);
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

				list<Matrix<double, 7, Dynamic>>::iterator pp;
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
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��imu���ݶ�ȡ��ʼ��\n");
				imuresult = Read(in.imu_data_file);
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
			vector < MatrixXd> results = imudetach_2(imuresult, total, rest);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("������������ɣ���%d��������\n", int(results.size()));
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			Get_anzhi_RotationMatrix(a, b, g);

			for (int i = 0; i < results.size(); i++)
			{
				list<Matrix<double, 7, Dynamic>> temp;
				list<Matrix<double, 7, Dynamic>>::iterator ap;
				for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
				{
					temp.push_back(*ap);
				}

				//���½���imu��lidarʱ��ƥ��
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				string sen;
				if (choice2 == 0)
				{
					sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ��ͬʱ����txt�ļ���\n";
					printf(sen.data());
					time_match_m_l_txt(results[i], temp, xx, yy, zz, (i + 1));
				}
				else
				{
					sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
					printf(sen.data());
					time_match_m_l(results[i], temp, xx, yy, zz);
				}
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
					writePcd_l(file, temp);
				}
				else if (0 == 2)
				{
					string file = "test" + to_string(i + 1) + ".ply";
					writePly_l(file, temp);
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
		}
		else
		{
			cout << "ģʽ��ƥ��" << endl;
		}
	}
	else if (model == 1)
	{
		if (choice == 1)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic >> lidar_final;
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ������ʼ��\n");
				lvxReader(in.lidar_data_file, lidar_final);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ����������\n");
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
				imuresult = Read_txt(in.imu_data_file);
			}
			catch (string mes)
			{
				mes += "\n";
				printf(mes.data());
				return 0;
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��imu���ݶ�ȡ������\n");
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			AGet_anzhi_RotationMatrix(a, b, g);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			if (choice2 == 0)
			{
				printf("����ʼʱ��ƥ��ͬʱ����txt�ļ���\n");
				Atime_match_m_l_txt(imufinal, lidar_final, xx, yy, zz);
			}
			else
			{
				printf("����ʼʱ��ƥ�䡣\n");
				Atime_match_m_l(imufinal, lidar_final, xx, yy, zz);
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��ʱ��ƥ����ɡ�\n");
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("����ʼ���ɵ����ļ�������\n");
			if (choice3 == 0)
			{
				string file = "test.las";
				AwriteLas_l(file, lidar_final);
			}
			else if (choice3 == 1)
			{
				string file = "test.pcd";
				AwritePcd_l(file, lidar_final);
			}
			else if (choice3 == 2)
			{
				string file = "test.ply";
				AwritePly_l(file, lidar_final);
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
		}
		else if (choice == 2)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic >> lidar_final;
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ������ʼ��\n");
				lvxReader(in.lidar_data_file, lidar_final);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ����������\n");
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
				imuresult = Read_txt(in.imu_data_file);
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
			printf("����ʼ�������롤����\n");
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
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("������������ɡ�\n");
			imuresult.clear();
			imuresult.shrink_to_fit();
			results.clear();
			results.shrink_to_fit();
			//���½���imu��lidarʱ��ƥ��
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			AGet_anzhi_RotationMatrix(a, b, g);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			if (choice2 == 0)
			{
				printf("����ʼʱ��ƥ��ͬʱ����txt�ļ���\n");
				Atime_match_m_l_txt(imufinal, lidar_final, xx, yy, zz);
			}
			else
			{
				printf("����ʼʱ��ƥ�䡣\n");
				Atime_match_m_l(imufinal, lidar_final, xx, yy, zz);
			}
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("��ʱ��ƥ����ɡ�\n");
			//�������ɵ����ļ�
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("����ʼ���ɵ����ļ�������\n");
			if (choice3 == 0)
			{
				string file = "test.las";
				AwriteLas_l(file, lidar_final);
			}
			else if (choice3 == 1)
			{
				string file = "test.pcd";
				AwritePcd_l(file, lidar_final);
			}
			else if (choice3 == 2)
			{
				string file = "test.ply";
				AwritePly_l (file, lidar_final);
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
		}
		else if (choice == 3)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic >> lidar_final;
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ������ʼ��\n");
				lvxReader(in.lidar_data_file, lidar_final);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ����������\n");
			}
			catch (string mes)
			{
				mes += "\n";
				printf(mes.data());
				return 0;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��imu���ݶ�ȡ��ʼ��\n");
				imuresult = Read_txt(in.imu_data_file);
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
			printf("����ʼ�������롤����\n");
			results = Aimudetach(imuresult, total, rest);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("������������ɣ���%d��������\n", results.size());
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			AGet_anzhi_RotationMatrix(a, b, g);
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
				if (choice2 == 1)
				{
					try
					{
						time(&now);
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						cout << tmp;
						sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						cout << sen;
						Atime_match_m_l(results[i], temp, xx, yy, zz);
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
					printf("��ʱ��ƥ����ɡ�\n");
					if (temp.size() == 0)
					{
						printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						continue;
					}
				}
				else
				{
					try
					{
						time(&now);
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						cout << tmp;
						sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						cout << sen;
						Atime_match_m_l_txt(results[i], temp, xx, yy, zz, (i + 1));
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
					printf("��ʱ��ƥ����ɡ�\n");
					if (temp.size() == 0)
					{
						printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						continue;
					}
				}
				//�������ɵ����ļ�
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				cout << sen;
				if (choice3 == 0)
				{
					string file = "test" + to_string(i + 1) + ".las";
					AwriteLas_l(file, temp);
				}
				else if (choice3 == 1)
				{
					string file = "test" + to_string(i + 1) + ".pcd";
					AwritePcd_l(file, temp);
				}
				else if (choice3 == 2)
				{
					string file = "test" + to_string(i + 1) + ".ply";
					AwritePly_l(file, temp);
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
			}
		}
		else if (choice == 4)
		{
			//���½���lidar���ݴ���
			list<Matrix<double, 6, Dynamic >> lidar_final;
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ������ʼ��\n");
				lvxReader(in.lidar_data_file, lidar_final);
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��lidar���ݶ�ȡ����������\n");
			}
			catch (string mes)
			{
				mes += "\n";
				printf(mes.data());
				return 0;
			}
			//���½���imu���ݴ���
			vector < Matrix<double, 7, 2000>> imuresult;
			vector < MatrixXd> results;
			try {
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				printf("��imu���ݶ�ȡ��ʼ��\n");
				imuresult = Read_txt(in.imu_data_file);
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
			printf("����ʼ�������롤����\n");
			results = Aimudetach_2(imuresult, total, rest);
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("������������ɣ���%d��������\n", results.size());
			imuresult.clear();
			imuresult.shrink_to_fit();
			time(&now);
			strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
			cout << tmp;
			printf("�����ð��þ���...\n");
			AGet_anzhi_RotationMatrix(a, b, g);
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
				if (choice2 == 1)
				{
					try
					{
						time(&now);
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						cout << tmp;
						sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
						cout << sen;
						Atime_match_m_l(results[i], temp, xx, yy, zz);
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
					printf("��ʱ��ƥ����ɡ�\n");
					if (temp.size() == 0)
					{
						printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						continue;
					}
				}
				else
				{
					try
					{
						time(&now);
						strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
						cout << tmp;
						sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
						cout << sen;
						Atime_match_m_l_txt(results[i], temp, xx, yy, zz, (i + 1));
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
					printf("��ʱ��ƥ����ɡ�\n");
					if (temp.size() == 0)
					{
						printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
						continue;
					}
				}
				//�������ɵ����ļ�
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
				cout << sen;
				if (choice3 == 0)
				{
					string file = "test" + to_string(i + 1) + ".las";
					AwriteLas_l(file, temp);
				}
				else if (choice3 == 1)
				{
					string file = "test" + to_string(i + 1) + ".pcd";
					AwritePcd_l(file, temp);
				}
				else if (choice3 == 2)
				{
					string file = "test" + to_string(i + 1) + ".ply";
					AwritePly_l(file, temp);
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
			}
		}
		else
		{
			cout << "ģʽ��ƥ��" << endl;
		}
	}
	else
	{
		cout << "�״��ͺŲ�����" << endl;
		return 0;
	}
	return 0;
}

//����
/*int main()
{
	time_t now;
	char tmp[16];
	int choice2 = 1;
	int choice3 = 0;
	double a, b, g, xx, yy, zz;
	a = 0;
	b = 0;
	g = 0;
	xx = 0;
	yy = 0;
	zz = 0;
	Interface::file_input in;
	in.lidar_data_file = "D:\\��Ŀ\\2021-07-26_18-02-36.lid";
	in.imu_data_file = "D:\\��Ŀ\\a1-40.txt";
	//���½���lidar���ݴ���
	list<Matrix<double, 6, Dynamic >> lidar_final;
	try
	{
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݶ�ȡ������ʼ��\n");
		lvxReader(in.lidar_data_file, lidar_final);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��lidar���ݶ�ȡ����������\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	//���½���imu���ݴ���
	vector < Matrix<double, 7, 2000>> imuresult;
	vector < MatrixXd> results;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("��imu���ݶ�ȡ��ʼ��\n");
		imuresult = Read_txt(in.imu_data_file);
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
	printf("����ʼ�������롤����\n");
	results = Aimudetach(imuresult, total, rest);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("������������ɣ���%d��������\n", results.size());
	imuresult.clear();
	imuresult.shrink_to_fit();
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("�����ð��þ���...\n");
	AGet_anzhi_RotationMatrix(a, b, g);
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
		if (choice2 == 1)
		{
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䡣\n";
				cout << sen;
				Atime_match_m_l(results[i], temp, xx, yy, zz);
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
			printf("��ʱ��ƥ����ɡ�\n");
			if (temp.size() == 0)
			{
				printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
				continue;
			}
		}
		else
		{
			try
			{
				time(&now);
				strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
				cout << tmp;
				sen = "���Ե�" + to_string(i + 1) + "��������ʼʱ��ƥ�䲢�������ݡ�\n";
				cout << sen;
				Atime_match_m_l_txt(results[i], temp, xx, yy, zz, (i + 1));
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
			printf("��ʱ��ƥ����ɡ�\n");
			if (temp.size() == 0)
			{
				printf("ƥ����Ϊ�գ����޶Եȵ�ʱ������ꡣ\n");
				continue;
			}
		}
		//�������ɵ����ļ�
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		sen = "���Ե�" + to_string(i + 1) + "��������ʼ����������ļ���\n";
		cout << sen;
		if (choice3 == 0)
		{
			string file = "test" + to_string(i + 1) + ".las";
			AwriteLas_l(file, temp);
		}
		else if (choice3 == 1)
		{
			string file = "test" + to_string(i + 1) + ".pcd";
			AwritePcd_l(file, temp);
		}
		else if (choice3 == 2)
		{
			string file = "test" + to_string(i + 1) + ".ply";
			AwritePly_l(file, temp);
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
	}
	return 0;
}*/

//����
/*int main()
{
	Matrix<double, 6, Dynamic > lidar;
	lidar.resize(6, 1);
	lidar(0, 0) = 0;
	lidar(1, 0) = 63.028999328613;
	lidar(2, 0) = 44.599998474121;
	lidar(3, 0) = -3.325999975204;
	lidar(4, 0) = 10;
	lidar(5, 0) = 1;
	AGet_anzhi_RotationMatrix(0, 0, 0);
	TIMEA::Act(lidar, 39.296180725098, 110.276763916016, 1299.807983398438, 0.183300003409, -3.985100030899, 85.328002929688, 0, 0, 0);
	cout << setprecision(14) << lidar(1, 0) << " " << lidar(2, 0) << " " << lidar(3, 0) << endl;
}*/