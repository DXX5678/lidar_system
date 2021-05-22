#ifndef _CON_H
#define _CON_H
#endif
#pragma once
#include <string>
using namespace std;
namespace conversion
{
	//�Ƕȹ淶��
	double mod_angle(double value);

	//����ת����
	double zone_number_to_central_longitude(int zone_number);

	//γ��ת����
	string lation_to_zone_letter(double latitude);

	//��γ��ת����
	int latlon_to_zone_number(double latitude, double longitude);

	//�����Լ��
	void check_valid_zone(int force_zone_number, string force_zone_letter);

	//��γ��ͶӰ
	bool from_latlon(double latitude, double longitude, double& easting, double& norting, int& zone_number, string& zone_letter, int force_zone_number = 0, string force_zone_letter = "");
}