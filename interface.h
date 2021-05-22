#ifndef _INTERFACE_H
#define _INTERFACE_H
#endif
#pragma once

#include <string>

using namespace std;

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

	/// <summary>
	/// ����һ����ȥ���յ� ���ֺ������ɵ����ļ�
	/// ����������ָ�� �ı�����ָ�� ����������ָ�� �����ļ��������� ����ļ��������� �״�������� ����ת����������
	/// </summary>
	void process_n_n(void *,void(*settext)(void *,char[100]),void(*setvalue)(void *,int), file_input, file_output, lidar_parameter, coordinate_parameter);

	/// <summary>
	/// ��������ȥ���յ� ���ֺ������ɵ����ļ�
	/// ����������ָ�� �ı�����ָ�� ����������ָ�� �����ļ��������� ����ļ��������� �״�������� ����ת����������
	/// </summary>
	void process_y_n(void*, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter);

	/// <summary>
	/// ��������ȥ���յ� �ֺ������ɵ����ļ�
	/// ����������ָ�� �ı�����ָ�� ����������ָ�� �����ļ��������� ����ļ��������� �״�������� ����ת����������
	/// </summary>
	void process_y_y(void*, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter);
}