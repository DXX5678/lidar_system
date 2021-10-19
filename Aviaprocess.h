#ifndef _AVIA_H
#define _AVIA_H
#endif
#pragma once
#include <vector>
#include <list>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
namespace Aviaissue
{
#define MAGIC_CODE       (0xac0ea767)
#define READ_BUFFER_LEN  1024 * 1024 * 32
#define TRIPLE_POINT_NUM  30
#pragma region Type Definations
#pragma pack(1)
	//lvx文件公共头
	typedef struct {
		uint8_t signature[16];
		uint8_t version[4];
		uint32_t magic_code;
	} LvxFilePublicHeader;

	//lvx文件私有头
	typedef struct {
		uint32_t frame_duration;
		uint8_t device_count;
	} LvxFilePrivateHeader;

	//lvx文件设备信息
	typedef struct {
		uint8_t lidar_broadcast_code[16];
		uint8_t hub_broadcast_code[16];
		uint8_t device_index;
		uint8_t device_type;
		uint8_t extrinsic_enable;
		float roll;
		float pitch;
		float yaw;
		float x;
		float y;
		float z;
	} LvxDeviceInfo;

	//lvx文件结构头
	typedef struct {
		uint64_t current_offset;
		uint64_t next_offset;
		uint64_t frame_index;
	} FrameHeader;

	//lvx文件数据包头
	typedef struct {
		uint8_t device_index;
		uint8_t version;
		uint8_t port_id;
		uint8_t lidar_index;
		uint8_t rsvd;
		uint32_t error_code;
		uint8_t timestamp_type;
		uint8_t data_type;
		uint8_t timestamp[4];
		uint32_t timestamp_us;
	} LvxBasePackHeader;


	//lvx文件数据信息
	typedef struct {
		int32_t x1;
		int32_t y1;
		int32_t z1;
		uint8_t r1;
		uint8_t f1;
		int32_t x2;
		int32_t y2;
		int32_t z2;
		uint8_t r2;
		uint8_t f2;
		int32_t x3;
		int32_t y3;
		int32_t z3;
		uint8_t r3;
		uint8_t f3;
	}Point;
#pragma pack()
#pragma endregion

	//获取文件字节大小
	size_t getFileSize_C(const char* file);

	//保留小数位操作
	double precision(double x, int y);

	//时间戳解析操作
	double timeconver(uint8_t timestamp[4], uint32_t timestamp_us);

	//预分块函数
	vector <long long int> prelvx(string filename);

	//解析lvx文件函数，返回lidar数据矩阵
	void lvxReader(string filename, list<Matrix<double, 6, Dynamic >>& lidar_final);
}