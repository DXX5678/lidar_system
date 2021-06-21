#ifndef _INTERFACE_H
#define _INTERFACE_H
#endif
#pragma once

#include <string>


//#ifndef MYTHREAD_H
#define MYTHREAD_H

//#include <QMainWindow>
//#include <QObject>
//#include<QThread>

//#include <QWidget>

using namespace std;

class mythread;

namespace Interface
{
	/// ˵�����֣�
	/// �����ļ�����  0-las  1-pcd  2-ply
	/// �״�����  0-RS16  1-RS32  2-Avia
	/// �״﹤��ģʽ  0-���ز�  1-˫�ز�  (AviaĬ��һ�ֹ���ģʽ0-���ز�)

	//�����ļ���������
	typedef struct file_input
	{
		string lidar_data_file; //�״������ļ���ַ(������׺��)
		string imu_data_file; //imu�����ļ���ַ(������׺��)
	}file_input;

	//����ļ���������
	typedef struct file_output
	{
		string points_file; //�����ļ������ַ(�޺�׺��)
		uint8_t points_file_type; //�����ļ�����
	}file_output;

	//�״��������
	typedef struct lidar_parameter
	{
		uint8_t lidar_type; //�״�����
		uint8_t lidar_work_model; //�״﹤��ģʽ
	}lidar_parameter;

	//����ת����������
	typedef struct coordinate_parameter
	{
		double x_offset; //ƽ����
		double y_offset;
		double z_offset;
		double alpha; //���ý�
		double beta;
		double gamma;
	}coordinate_parameter;

	void testfuc();
	/// <summary>
	/// ����һ����ȥ���յ� ���ֺ������ɵ����ļ�
	/// ����������ָ�� �ı�����ָ�� ����������ָ�� �����ļ��������� ����ļ��������� �״�������� ����ת���������� �̸߳���
	/// </summary>
	void process_n_n(mythread*, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter, int num);

	/// <summary>
	/// ��������ȥ���յ� ���ֺ������ɵ����ļ�
	/// ����������ָ�� �ı�����ָ�� ����������ָ�� �����ļ��������� ����ļ��������� �״�������� ����ת���������� �̸߳���
	/// </summary>
	void process_y_n(mythread*, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter, int num);

	/// <summary>
	/// ��������ȥ���յ� �ֺ������ɵ����ļ�
	/// ����������ָ�� �ı�����ָ�� ����������ָ�� �����ļ��������� ����ļ��������� �״�������� ����ת���������� �̸߳���
	/// </summary>
	void process_y_y(mythread*, void(mythread::* settext)(void*, char[100]), void(mythread::* setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter, int num);
}

/*
class mythread : public QThread
{
	Q_OBJECT
public:
	mythread();
	~mythread();

	int barprocess;
	string textshow;
	void changebarprocess(void* a, int b);
	void changetextshow(void* a, char b[100]);

	void* mainsting;
	void(mythread::* settext)(void*, char[100]);
	void(mythread::* setvalue)(void*, int);
	Interface::file_input file_input_thread;
	Interface::file_output file_output_thread;
	Interface::lidar_parameter lidar__parameter_thread;
	Interface::coordinate_parameter coordinate_parameter_thread;

protected:
	//QThread���麯��
	//�̴߳�����
	//����ֱ�ӵ��ã�ͨ��start()��ӵ���
	void run();

	//public slots:
		//void testthread();

signals:
	void isDone();
	void intchange();
	void charchange();

};
*/