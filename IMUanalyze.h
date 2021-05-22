#ifndef _CON_H
#define _CON_H
#endif
#pragma once
#include <string>
using namespace std;
namespace conversion
{
	//角度规范化
	double mod_angle(double value);

	//区域转经度
	double zone_number_to_central_longitude(int zone_number);

	//纬度转区域
	string lation_to_zone_letter(double latitude);

	//经纬度转区域
	int latlon_to_zone_number(double latitude, double longitude);

	//合理性检查
	void check_valid_zone(int force_zone_number, string force_zone_letter);

	//经纬度投影
	bool from_latlon(double latitude, double longitude, double& easting, double& norting, int& zone_number, string& zone_letter, int force_zone_number = 0, string force_zone_letter = "");
}