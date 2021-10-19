#include "Aviaprocess.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <atltime.h>
#include <ctime>
#include <vector>
#include <io.h>
#include <ctime>
#include<string.h>
#include<list>
#pragma warning(disable:4996)
using namespace std;
using namespace Eigen;

namespace Aviaissue

{
	size_t getFileSize_C(const char* file)
	{
		size_t size;
		ifstream is;
		is.open(file, ios::binary);
		if (!is.is_open())
		{
			throw"lidar文件打开失败！";
		}
		else
		{

			is.seekg(0, ios::end);//设置流文件指针的位置,以SEEK_END为起点，偏移量是0,亦即SEEK_END
			size = is.tellg();//函数结果：当前文件流指针位置相对于文件起始位置的字节偏移量
			is.close();
		}

		return size;
	}

	double precision(double x, int y = 3)
	{
		double temp = pow(0.1, y + 1);
		x += temp * 5;
		int temp2 = pow(10, y);
		long long int m = floor(x * temp2);
		x = (double)m / temp2;
		return x;
	}

	double timeconver(uint8_t timestamp[4], uint32_t timestamp_us)
	{
		int year = int(timestamp[0]);
		if (year < 2000)
		{
			year += 2000;
		}
		int month = int(timestamp[1]);
		int day = int(timestamp[2]);
		int hour = int(timestamp[3]);
		int us = int(timestamp_us);
		CTime time(year, month, day, 0, 0, 0);
		int week = time.GetDayOfWeek() - 1;
		double timeoffset = week * 86400 + hour * 3600 + (double)us/ 1000000;//86400
		return timeoffset;
	}

	vector<long long int> prelvx(string filename)
	{
		vector<long long int> number;
		size_t file_size;
		try
		{
			file_size = getFileSize_C(filename.data());
		}
		catch (const char* c)
		{
			throw c;
		}
		vector<LvxDeviceInfo> device_info_vec_;
		unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);
		fstream fp;
		fp.open(filename, ios::in | ios::binary);
		if (!fp.is_open())
		{
			throw"文件打开失败！";
		}

		uint64_t cur_offset_ = 0;
		LvxFilePublicHeader lvx_file_public_header_;
		LvxFilePrivateHeader lvx_file_private_header_;
		int header_size = sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader);
		fp.read((char*)read_buffer.get(), header_size);
		memcpy(&lvx_file_public_header_, read_buffer.get() + cur_offset_, sizeof(LvxFilePublicHeader));
		cur_offset_ += sizeof(LvxFilePublicHeader);
		if (lvx_file_public_header_.magic_code != MAGIC_CODE)
		{
			throw"lvx文件校验码错误！";
		}

		memcpy(&lvx_file_private_header_, read_buffer.get() + cur_offset_, sizeof(LvxFilePrivateHeader));
		cur_offset_ += sizeof(LvxFilePrivateHeader);


		int device_num = lvx_file_private_header_.device_count;
		device_info_vec_.resize(device_num);
		int device_info_size = device_num * sizeof(LvxDeviceInfo);
		fp.read((char*)read_buffer.get() + cur_offset_, device_info_size);
		for (int i = 0; i < device_num; ++i)
		{
			memcpy(&device_info_vec_[i], read_buffer.get() + cur_offset_, sizeof(LvxDeviceInfo));
			cur_offset_ += sizeof(LvxDeviceInfo);
		}
		long long int count = 0;
		double time1, time2;
		FrameHeader frame_header;
		LvxBasePackHeader lvx_basepacket_header;
		Point data;
		do
		{
			fp.read((char*)read_buffer.get(), sizeof(FrameHeader));
			memcpy(&frame_header, read_buffer.get(), sizeof(FrameHeader));
			if (frame_header.current_offset == 0)
			{
				break;
			}
			if (cur_offset_ != frame_header.current_offset)
			{
				throw"lvx文件读取FrameHeader出错！";
			}
			cur_offset_ += sizeof(FrameHeader);

			while (cur_offset_ < frame_header.next_offset)
			{
				fp.read((char*)read_buffer.get(), sizeof(LvxBasePackHeader));
				memcpy(&lvx_basepacket_header, read_buffer.get(), sizeof(LvxBasePackHeader));
				cur_offset_ += sizeof(LvxBasePackHeader);

				if (lvx_basepacket_header.timestamp_type != 3 || lvx_basepacket_header.error_code == 0)
				{
					uint32_t points_byte_size = sizeof(Point);
					for (int n = 0; n < TRIPLE_POINT_NUM; n++)
					{
						fp.read((char*)read_buffer.get(), points_byte_size);
						memcpy(&data, read_buffer.get(), points_byte_size);
						cur_offset_ += points_byte_size;
					}
					continue;
				}
				double timestamp = timeconver(lvx_basepacket_header.timestamp, lvx_basepacket_header.timestamp_us);
				time1 = timestamp;
				if (lvx_basepacket_header.data_type == 7)
				{
					uint32_t points_byte_size = sizeof(Point);
					for (int n = 0; n < TRIPLE_POINT_NUM; n++)
					{
						fp.read((char*)read_buffer.get(), points_byte_size);
						memcpy(&data, read_buffer.get(), points_byte_size);
						cur_offset_ += points_byte_size;
						int f1_echo = ((data.f1 & 0x30) >> 4);
						int f1_intensity = ((data.f1 & 0x0c) >> 2);
						int f1_position = (data.f1 & 0x03);
						int f2_echo = ((data.f2 & 0x30) >> 4);
						int f2_intensity = ((data.f2 & 0x0c) >> 2);
						int f2_position = (data.f2 & 0x03);
						int f3_echo = ((data.f3 & 0x30) >> 4);
						int f3_intensity = ((data.f3 & 0x0c) >> 2);
						int f3_position = (data.f3 & 0x03);
						double x1, y1, z1, x2, y2, z2, x3, y3, z3;
						if (f1_echo != 0 && f1_intensity == 0 && f1_position == 0)
						{
							x1 = precision((double(data.x1) / 1000));
							y1 = precision((double(data.y1) / 1000));
							z1 = precision((double(data.z1) / 1000));
							if ((x1 != 0 || y1 != 0 || z1 != 0) && x1 < 250 && x1 > 5)
							{
								time2 = precision(timestamp + (double(n * 3) + 0) * 1 / 720000);
								if (time1 != time2)
								{
									if (count == 0)
									{
										number.push_back(1);
										count = 1;
										time1 = time2;
									}
									else
									{
										number.push_back(count);
										count = 1;
										time1 = time2;
									}
								}
								else
								{
									++count;
								}
							}
						}
						if (f2_echo != 0 && f2_intensity == 0 && f2_position == 0)
						{
							x2 = precision((double(data.x2) / 1000));
							y2 = precision((double(data.y2) / 1000));
							z2 = precision((double(data.z2) / 1000));
							if ((x2 != 0 || y2 != 0 || z2 != 0) && x2 < 250 && x2 > 5)
							{
								time2 = precision(timestamp + (double(n * 3) + 1) * 1 / 720000);
								if (time1 != time2)
								{
									if (count == 0)
									{
										number.push_back(1);
										count = 1;
										time1 = time2;
									}
									else
									{
										number.push_back(count);
										count = 1;
										time1 = time2;
									}
								}
								else
								{
									++count;
								}
							}
						}
						if (f3_echo != 0 && f3_intensity == 0 && f3_position == 0)
						{
							x3 = precision((double(data.x3) / 1000));
							y3 = precision((double(data.y3) / 1000));
							z3 = precision((double(data.z3) / 1000));
							if ((x3 != 0 || y3 != 0 || z3 != 0) && x3 < 250 && x3 > 5)
							{
								time2 = precision(timestamp + (double(n * 3) + 2) * 1 / 720000);
								if (time1 != time2)
								{
									if (count == 0)
									{
										number.push_back(1);
										count = 1;
										time1 = time2;
									}
									else
									{
										number.push_back(count);
										count = 1;
										time1 = time2;
									}
								}
								else
								{
									++count;
								}
							}
						}
					}
				}
				else
				{
					throw"lvx文件读取DataType出错！";
				}
			}
			if (cur_offset_ != frame_header.next_offset)
			{
				throw"lvx文件读取PointData出错！";
			}

		} while (cur_offset_ != file_size);
		number.push_back(count);
		count = 0;
		return number;
	}

	void lvxReader(string filename, list<Matrix<double, 6, Dynamic >>& lidar_final)
	{
		vector<long long int> number = prelvx(filename);
		Matrix<double, 6, Dynamic> temp;
		for (long long int i = 0; i < number.size(); i++)
		{
			temp.resize(6, number[i]);
			lidar_final.push_back(temp);
		}
		size_t file_size;
		try
		{
			file_size = getFileSize_C(filename.data());
		}
		catch (const char* c)
		{
			throw c;
		}
		vector<LvxDeviceInfo> device_info_vec_;
		unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);
		fstream fp;
		fp.open(filename, ios::in | ios::binary);
		if (!fp.is_open())
		{
			throw"文件打开失败！";
		}

		uint64_t cur_offset_ = 0;
		LvxFilePublicHeader lvx_file_public_header_;
		LvxFilePrivateHeader lvx_file_private_header_;
		int header_size = sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader);
		fp.read((char*)read_buffer.get(), header_size);
		memcpy(&lvx_file_public_header_, read_buffer.get() + cur_offset_, sizeof(LvxFilePublicHeader));
		cur_offset_ += sizeof(LvxFilePublicHeader);
		if (lvx_file_public_header_.magic_code != MAGIC_CODE)
		{
			throw"lvx文件校验码错误！";
		}

		memcpy(&lvx_file_private_header_, read_buffer.get() + cur_offset_, sizeof(LvxFilePrivateHeader));
		cur_offset_ += sizeof(LvxFilePrivateHeader);


		int device_num = lvx_file_private_header_.device_count;
		device_info_vec_.resize(device_num);
		int device_info_size = device_num * sizeof(LvxDeviceInfo);
		fp.read((char*)read_buffer.get() + cur_offset_, device_info_size);
		for (int i = 0; i < device_num; ++i)
		{
			memcpy(&device_info_vec_[i], read_buffer.get() + cur_offset_, sizeof(LvxDeviceInfo));
			cur_offset_ += sizeof(LvxDeviceInfo);
		}

		FrameHeader frame_header;
		LvxBasePackHeader lvx_basepacket_header;
		long long int tt = 0;
		list<Matrix<double, 6, Dynamic>>::iterator id = lidar_final.begin();
		Point data;
		do
		{
			fp.read((char*)read_buffer.get(), sizeof(FrameHeader));
			memcpy(&frame_header, read_buffer.get(), sizeof(FrameHeader));
			if (frame_header.current_offset == 0)
			{
				/*
				cur_offset_ += sizeof(FrameHeader);
				fp.read((char*)read_buffer.get(), sizeof(LvxBasePackHeader));
				memcpy(&lvx_basepacket_header, read_buffer.get(), sizeof(LvxBasePackHeader));
				cur_offset_ += sizeof(LvxBasePackHeader);
				uint32_t points_byte_size = sizeof(Point);
				for (int n = 0; n < TRIPLE_POINT_NUM; n++)
				{
					fp.read((char*)read_buffer.get(), points_byte_size);
					memcpy(&data, read_buffer.get(), points_byte_size);
					cur_offset_ += points_byte_size;
				}
				continue;
				*/
				break;
			}
			if (cur_offset_ != frame_header.current_offset)
			{
				throw"lvx文件读取FrameHeader出错！";
			}
			cur_offset_ += sizeof(FrameHeader);
			
			while (cur_offset_ < frame_header.next_offset)
			{
				fp.read((char*)read_buffer.get(), sizeof(LvxBasePackHeader));
				memcpy(&lvx_basepacket_header, read_buffer.get(), sizeof(LvxBasePackHeader));
				cur_offset_ += sizeof(LvxBasePackHeader);

				if (lvx_basepacket_header.timestamp_type != 3 || lvx_basepacket_header.error_code == 0)
				{
					uint32_t points_byte_size = sizeof(Point);
					for (int n = 0; n < TRIPLE_POINT_NUM; n++)
					{
						fp.read((char*)read_buffer.get(), points_byte_size);
						memcpy(&data, read_buffer.get(), points_byte_size);
						cur_offset_ += points_byte_size;
					}
					continue;
				}
				double timestamp = timeconver(lvx_basepacket_header.timestamp, lvx_basepacket_header.timestamp_us);

				if (lvx_basepacket_header.data_type == 7)
				{

					uint32_t points_byte_size = sizeof(Point);
					
					for (int n = 0; n < TRIPLE_POINT_NUM; n++)
					{
						fp.read((char*)read_buffer.get(), points_byte_size);
						memcpy(&data, read_buffer.get(), points_byte_size);
						cur_offset_ += points_byte_size;
						int f1_echo = ((data.f1 & 0x30) >> 4);
						int f1_intensity = ((data.f1 & 0x0c) >> 2);
						int f1_position = (data.f1 & 0x03);
						int f2_echo = ((data.f2 & 0x30) >> 4);
						int f2_intensity = ((data.f2 & 0x0c) >> 2);
						int f2_position = (data.f2 & 0x03);
						int f3_echo = ((data.f3 & 0x30) >> 4);
						int f3_intensity = ((data.f3 & 0x0c) >> 2);
						int f3_position = (data.f3 & 0x03);
						double x1, y1, z1, x2, y2, z2, x3, y3, z3;
						if (f1_echo != 0 && f1_intensity == 0 && f1_position == 0)
						{
						
							x1 = precision((double(data.x1) / 1000));
							y1 = precision((double(data.y1) / 1000));
							z1 = precision((double(data.z1) / 1000));
							if ((x1 != 0 || y1 != 0 || z1 != 0) && x1 < 250 && x1 > 5)
							{
								if (tt <= (*id).cols() - 1)
								{
									(*id)(0, tt) = precision(timestamp + (double(n * 3) + 0) * 1 / 720000);
									(*id)(1, tt) = x1;
									(*id)(2, tt) = y1;
									(*id)(3, tt) = z1;
									(*id)(4, tt) = double(int(data.r1));
									(*id)(5, tt) = f1_echo;
									tt++;
								}
								else
								{
									id++;
									tt = 0;
									(*id)(0, tt) = precision(timestamp + (double(n * 3) + 0) * 1 / 720000);
									(*id)(1, tt) = x1;
									(*id)(2, tt) = y1;
									(*id)(3, tt) = z1;
									(*id)(4, tt) = double(int(data.r1));
									(*id)(5, tt) = f1_echo;
									tt++;
								}
							}
						}
						if (f2_echo != 0 && f2_intensity == 0 && f2_position == 0)
						{
						
							x2 = precision((double(data.x2) / 1000));
							y2 = precision((double(data.y2) / 1000));
							z2 = precision((double(data.z2) / 1000));
							if ((x2 != 0 || y2 != 0 || z2 != 0) && x2 < 250 && x2 > 5)
							{
								if (tt <= (*id).cols() - 1)
								{
									(*id)(0, tt) = precision(timestamp + (double(n * 3) + 1) * 1 / 720000);
									(*id)(1, tt) = x2;
									(*id)(2, tt) = y2;
									(*id)(3, tt) = z2;
									(*id)(4, tt) = double(int(data.r2));
									(*id)(5, tt) = f2_echo;
									tt++;
								}
								else
								{
									id++;
									tt = 0;
									(*id)(0, tt) = precision(timestamp + (double(n * 3) + 1) * 1 / 720000);
									(*id)(1, tt) = x2;
									(*id)(2, tt) = y2;
									(*id)(3, tt) = z2;
									(*id)(4, tt) = double(int(data.r2));
									(*id)(5, tt) = f2_echo;
									tt++;
								}
							}
						}
						if (f3_echo != 0 && f3_intensity == 0 && f3_position == 0)
						{
						
							x3 = precision((double(data.x3) / 1000));
							y3 = precision((double(data.y3) / 1000));
							z3 = precision((double(data.z3) / 1000));
							if ((x3 != 0 || y3 != 0 || z3 != 0) && x3 < 250 && x3 > 5)
							{
								if (tt <= (*id).cols() - 1)
								{
									(*id)(0, tt) = precision(timestamp + (double(n * 3) + 2) * 1 / 720000);
									(*id)(1, tt) = x3;
									(*id)(2, tt) = y3;
									(*id)(3, tt) = z3;
									(*id)(4, tt) = double(int(data.r3));
									(*id)(5, tt) = f3_echo;
									tt++;
								}
								else
								{
									id++;
									tt = 0;
									(*id)(0, tt) = precision(timestamp + (double(n * 3) + 2) * 1 / 720000);
									(*id)(1, tt) = x3;
									(*id)(2, tt) = y3;
									(*id)(3, tt) = z3;
									(*id)(4, tt) = double(int(data.r3));
									(*id)(5, tt) = f3_echo;
									tt++;
								}
							}
						}
					}
				}
				else if (lvx_basepacket_header.data_type == 8)
				{

				}
				else if (lvx_basepacket_header.data_type == 0)
				{

				}
				else if (lvx_basepacket_header.data_type == 1)
				{

				}
				else if (lvx_basepacket_header.data_type == 2)
				{

				}
				else if (lvx_basepacket_header.data_type == 3)
				{

				}
				else if (lvx_basepacket_header.data_type == 4)
				{

				}
				else if (lvx_basepacket_header.data_type == 5)
				{

				}
				else if (lvx_basepacket_header.data_type == 6)
				{

				}
				else
				{
					throw"lvx文件读取DataType出错！";
				}
			}
			if (cur_offset_ != frame_header.next_offset)
			{
				throw"lvx文件读取PointData出错！";
			}

		} while (cur_offset_ != file_size);
	}
}
