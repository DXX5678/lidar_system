#ifndef _INTERFACE_H
#define _INTERFACE_H
#endif
#pragma once

#include <string>

using namespace std;

namespace Interface
{
	/// 说明部分：
	/// 点云文件类型  0-las  1-pcd  2-ply
	/// 雷达类型  0-RS16  1-RS32  2-Avia
	/// 雷达工作模式  0-单回波  1-双回波  (Avia默认一种工作模式0-三回波)

	//输入文件参数设置
	typedef struct file_input
	{
		string lidar_data_file; //雷达数据文件地址(包含后缀名)
		string imu_data_file; //imu数据文件地址(包含后缀名)
	}file_input;

	//输出文件参数设置
	typedef struct file_output
	{
		string points_file; //点云文件保存地址(无后缀名)
		uint8_t points_file_type; //点云文件类型
	}file_output;

	//雷达参数设置
	typedef struct lidar_parameter
	{
		uint8_t lidar_type; //雷达类型
		uint8_t lidar_work_model; //雷达工作模式
	}lidar_parameter;

	//坐标转换参数设置
	typedef struct coordinate_parameter
	{
		double x_offset; //平移量
		double y_offset;
		double z_offset;
		double alpha; //安置角
		double beta;
		double gamma;
	}coordinate_parameter;

	/// <summary>
	/// 函数一：不去除拐点 不分航带生成点云文件
	/// 参数：对象指针 文本函数指针 进度条函数指针 输入文件参数设置 输出文件参数设置 雷达参数设置 坐标转换参数设置
	/// </summary>
	void process_n_n(void *,void(*settext)(void *,char[100]),void(*setvalue)(void *,int), file_input, file_output, lidar_parameter, coordinate_parameter);

	/// <summary>
	/// 函数二：去除拐点 不分航带生成点云文件
	/// 参数：对象指针 文本函数指针 进度条函数指针 输入文件参数设置 输出文件参数设置 雷达参数设置 坐标转换参数设置
	/// </summary>
	void process_y_n(void*, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter);

	/// <summary>
	/// 函数三：去除拐点 分航带生成点云文件
	/// 参数：对象指针 文本函数指针 进度条函数指针 输入文件参数设置 输出文件参数设置 雷达参数设置 坐标转换参数设置
	/// </summary>
	void process_y_y(void*, void(*settext)(void*, char[100]), void(*setvalue)(void*, int), file_input, file_output, lidar_parameter, coordinate_parameter);
}