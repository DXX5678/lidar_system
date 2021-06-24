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
	//以下进行imu数据处理
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：imu数据读取开始。\n");
		imuresult = Read("sbet_Mission 1.out");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：imu数据读取结束。\n");
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
	printf("：imu数据插值开始。\n");
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
	printf("：imu数据插值结束。\n");
	
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

/*
int main()
{
	time_t now;
	char tmp[16];
	int num;
	cout << "用户输入线程数：";
	cin >> num;
	//以下进行lidar数据处理
	list<Matrix<double, 6, Dynamic>> lidar_final;
	list<vector<string>>reading;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：lidar数据读取开始。\n");
		reading = lidar_reading("RS32_2021-03-13_10-55-42-633088.tianyu");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：lidar数据读取结束。\n");

		
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
		printf("：lidar数据解析开始。\n");
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
		printf("：lidar数据解析结束。\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	//以下进行imu数据处理
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：imu数据读取开始。\n");
		imuresult = Read("sbet_Mission_shu.dat");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：imu数据读取结束。\n");
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
	printf("：imu数据插值开始。\n");
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
	printf("：imu数据插值结束。\n");
	//以下进行imu和lidar时间匹配
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：设置安置矩阵...\n");
	Get_anzhi_RotationMatrix(0, 0, 0);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：开始时间匹配。\n");
	time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：时间匹配完成。\n");
	//以下生成点云文件
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：开始生成点云文件···\n");
	if (2 == 0)
	{
		string file = "test.las";
		writeLas_l(file, lidar_final);
	}
	else if (2 == 1)
	{
		string file = "test.pcd";
		writePcd_l(file, lidar_final);
	}
	else if (2 == 2)
	{
		string file = "test.ply";
		writePly_l(file, lidar_final);
	}
	else
	{
		printf("输出文件类型选择错误！\n");
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：点云文件生成完成。\n");
	return 0;
}
*/

int main()
{
	time_t now;
	char tmp[16];
	int num;
	cout << "用户输入线程数：";
	cin >> num;
	//以下进行lidar数据处理
	list<Matrix<double, 6, Dynamic>> lidar_final;
	list<vector<string>>reading;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout<<"：lidar数据读取开始。\n";
		reading = lidar_reading("RS32_2021-03-13_10-55-42-633088.tianyu");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout<<"：lidar数据读取结束。\n";
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
		cout << "：lidar数据解析开始。\n";
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
		cout << "：lidar数据解析结束。\n";
	}
	catch (string mes)
	{
		mes += "\n";
		cout << mes;
		return 0;
	}
	//以下进行imu数据处理
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "：imu数据读取开始。\n";
		imuresult = Read("sbet_Mission_shu.dat");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		cout << "：imu数据读取结束。\n";
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
	cout << "：imu数据插值开始。\n";
	imu_timenew(imuresult);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：imu数据插值结束。\n";
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：开始航带分离···\n";
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
	cout << "：航带分离完成。\n";
	imuresult.clear();
	imuresult.shrink_to_fit();
	results.clear();
	results.shrink_to_fit();

	//以下进行imu和lidar时间匹配
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：设置安置矩阵...\n";
	Get_anzhi_RotationMatrix(0, 0, 0);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：开始时间匹配。\n";
	time_match_m_l(imufinal, lidar_final, 0.0758, -0.038, -0.002);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：时间匹配完成。\n";
	//以下生成点云文件
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：开始生成点云文件···\n";
	if (2 == 0)
	{
		string file =  "tests.las";
		writeLas_l(file, lidar_final);
	}
	else if (2 == 1)
	{
		string file = "tests.pcd";
		writePcd_l(file, lidar_final);
	}
	else if (2 == 2)
	{
		string file = "tests.ply";
		writePly_l(file, lidar_final);
	}
	else
	{
		cout << "输出文件类型选择错误！\n";
		return 0;
	}
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	cout << "：点云文件生成完成。\n";
	return 0;
}

/*
int main()
{
	time_t now;
	char tmp[16];
	int num;
	cout << "用户输入线程数：";
	cin >> num;
	//以下进行lidar数据处理
	list<Matrix<double, 6, Dynamic>> lidar_final;
	list<vector<string>>reading;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：lidar数据读取开始。\n");
		reading = lidar_reading("RS32_2021-03-13_10-55-42-633088.tianyu");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：lidar数据读取结束。\n");


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
		printf("：lidar数据解析开始。\n");
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
		printf("：lidar数据解析结束。\n");
	}
	catch (string mes)
	{
		mes += "\n";
		printf(mes.data());
		return 0;
	}
	//以下进行imu数据处理
	vector < Matrix<double, 7, 2000>> imuresult;
	MatrixXd imufinal;
	try {
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：imu数据读取开始。\n");
		imuresult = Read("sbet_Mission_shu.dat");
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：imu数据读取结束。\n");
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
	printf("：imu数据插值开始。\n");
	imu_timenew(imuresult);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：imu数据插值结束。\n");
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：开始航带分离···\n");
	vector < MatrixXd> results = imudetach(imuresult, total, rest);
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：航带分离完成。\n");
	imuresult.clear();
	imuresult.shrink_to_fit();
	time(&now);
	strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
	cout << tmp;
	printf("：设置安置矩阵...\n");
	Get_anzhi_RotationMatrix(0, 0, 0);

	for (int i = 0; i < results.size(); i++)
	{
		list<Matrix<double, 6, Dynamic>> temp;
		list<Matrix<double, 6, Dynamic>>::iterator ap;
		for (ap = lidar_final.begin(); ap != lidar_final.end(); ap++)
		{
			temp.push_back(*ap);
		}
		//以下进行imu和lidar时间匹配
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		string sen = "：对第" + to_string(i + 1) + "条航带开始时间匹配。\n";
		printf(sen.data());
		time_match_m_l(results[i], temp, 0.0758, -0.038, -0.002);
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：时间匹配完成。\n");
		if (temp.size() == 0)
		{
			printf("匹配结果为空，即无对等的时间戳坐标。\n");
			continue;
		}
		//以下生成点云文件
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		sen = "：对第" + to_string(i + 1) + "条航带开始生成其点云文件。\n";
		printf(sen.data());
		if (2 == 0)
		{
			string file = "test" + to_string(i + 1) + ".las";
			writeLas_l(file, temp);
		}
		else if (2 == 1)
		{
			string file = "test" + to_string(i + 1) + ".pcd";
			writePcd_l(file, temp);
		}
		else if (2 == 2)
		{
			string file = "test" + to_string(i + 1) + ".ply";
			writePly_l(file, temp);
		}
		else
		{
			printf("输出文件类型选择错误！\n");
			return 0;
		}
		time(&now);
		strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&now));
		cout << tmp;
		printf("：点云文件生成完成。\n");
		temp.clear();
	}
	return 0;
}
*/