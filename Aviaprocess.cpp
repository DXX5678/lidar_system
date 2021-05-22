#include "Aviaprocess.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <atltime.h>
#include <ctime>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Aviaissue
{
	size_t getFileSize_C(const char* file)
	{
		size_t size = -1;
		FILE* path;
		path = fopen(file, "r");
		if (NULL == path)
		{
			throw"lidar文件打开失败！";
		}
		else
		{
			fseek(path, 0, SEEK_END);//设置流文件指针的位置,以SEEK_END为起点，偏移量是0,亦即SEEK_END
			size = ftell(path);//函数结果：当前文件流指针位置相对于文件起始位置的字节偏移量
			fclose(path);
			path = NULL;
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

	double timeconver(uint8_t timestamp[8])
	{
		int year = int(timestamp[0]);
		if (year < 2000)
		{
			year += 2000;
		}
		int month = int(timestamp[1]);
		int day = int(timestamp[2]);
		int hour = int(timestamp[3]);
		string offset;
		for (int i = 4; i < 8; i++)
		{
			offset += timestamp[i];
		}
		double us = double(atoi(offset.data()));
		CTime time(year, month, day, 0, 0, 0);
		int week = time.GetDayOfWeek() - 1;
		double timeoffset = week * 86400 + hour * 3600 + us / 1000000;
		return timeoffset;
	}

	MatrixXd lvxReader(string filename)
	{
		size_t file_size;
		try
		{
			file_size = getFileSize_C(filename.data());
		}
		catch (string mes)
		{
			throw mes;
		}
		vector<LvxDeviceInfo> device_info_vec_;
		vector<MatrixXd> points;
		MatrixXd result;
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
		Point data;
		do
		{
			fp.read((char*)read_buffer.get(), sizeof(FrameHeader));
			memcpy(&frame_header, read_buffer.get(), sizeof(FrameHeader));
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

				double timestamp = timeconver(lvx_basepacket_header.timestamp);

				if (lvx_basepacket_header.data_type == 0)
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
				else if (lvx_basepacket_header.data_type == 7)
				{

					uint32_t points_byte_size = sizeof(Point);
					for (int n = 0; n < TRIPLE_POINT_NUM; n++)
					{
						fp.read((char*)read_buffer.get(), points_byte_size);
						memcpy(&data, read_buffer.get(), points_byte_size);
						cur_offset_ += points_byte_size;
						MatrixXd temp;
						int f1_echo = ((data.f1 >> 4) & 3);
						int f1_intensity = ((data.f1 >> 2) & 3);
						int f1_position = (data.f1 & 3);
						int f2_echo = ((data.f2 >> 4) & 3);
						int f2_intensity = ((data.f2 >> 2) & 3);
						int f2_position = (data.f2 & 3);
						int f3_echo = ((data.f3 >> 4) & 3);
						int f3_intensity = ((data.f3 >> 2) & 3);
						int f3_position = (data.f3 & 3);
						double x1, y1, z1, x2, y2, z2, x3, y3, z3;
						if (f1_echo != 0 && f1_intensity == 0 && f1_position == 0)
						{
							temp.resize(6, 1);
							x1 = precision((double(data.x1) / 1000));
							y1 = precision((double(data.y1) / 1000));
							z1 = precision((double(data.z1) / 1000));
							temp(0, 0) = precision((timestamp + double(n * 3) / 1000000));
							temp(1, 0) = x1;
							temp(2, 0) = y1;
							temp(3, 0) = z1;
							temp(4, 0) = double(int(data.r1));
							temp(5, 0) = f1_echo;
							points.push_back(temp);
						}
						if (f2_echo != 0 && f2_intensity == 0 && f2_position == 0)
						{
							temp.resize(6, 1);
							x2 = precision((double(data.x2) / 1000));
							y2 = precision((double(data.y2) / 1000));
							z2 = precision((double(data.z2) / 1000));
							temp(0, 0) = precision((timestamp + double((n * 3) + 1) / 1000000));
							temp(1, 0) = x2;
							temp(2, 0) = y2;
							temp(3, 0) = z2;
							temp(4, 0) = double(int(data.r2));
							temp(5, 0) = f2_echo;
							points.push_back(temp);
						}
						if (f3_echo != 0 && f3_intensity == 0 && f3_position == 0)
						{
							temp.resize(6, 1);
							x3 = precision((double(data.x3) / 1000));
							y3 = precision((double(data.y3) / 1000));
							z3 = precision((double(data.z3) / 1000));
							temp(0, 0) = precision((timestamp + double((n * 3) + 2) / 1000000));
							temp(1, 0) = x3;
							temp(2, 0) = y3;
							temp(3, 0) = z3;
							temp(4, 0) = double(int(data.r3));
							temp(5, 0) = f3_echo;
							points.push_back(temp);
						}
					}
				}
				else if (lvx_basepacket_header.data_type == 8)
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
		long long int i = 0;
		for (; i < points.size(); i += 1)
		{
			result.conservativeResize(6, (i * 1) + 1);
			result.block(0, i, 6, 1) = points[i];
		}
		return result;
	}
}