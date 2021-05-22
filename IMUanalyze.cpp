#include <math.h>
#include <algorithm>
#include <string>
#include <corecrt_math_defines.h>
using namespace std;

/*用于经纬度投影*/
namespace conversion {
	double Rad_to_deg = 45.0 / atan(1.0);
	double K0 = 0.9996;
	double E = 0.00669438;
	double E2 = E * E;
	double E3 = E2 * E;
	double E_P2 = E / (1.0 - E);
	double SQRT_E = sqrt(1 - E);
	double _E = (1 - SQRT_E) / (1 + SQRT_E);
	double _E2 = _E * _E;
	double _E3 = _E2 * _E;
	double _E4 = _E3 * _E;
	double _E5 = _E4 * _E;
	double M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256);
	double M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024);
	double M3 = (15 * E2 / 256 + 45 * E3 / 1024);
	double M4 = (35 * E3 / 3072);
	double P2 = (3.0 / 2 * _E - 27.0 / 32 * _E3 + 269.0 / 512 * _E5);
	double P3 = (21.0 / 16 * _E2 - 55.0 / 32 * _E4);
	double P4 = (151.0 / 96 * _E3 - 417.0 / 128 * _E5);
	double P5 = (1097.0 / 512 * _E4);
	int R = 6378137;
	string ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX";
	double mod_angle(double value)
	{
		return (fmod(value + M_PI, 2.0 * M_PI) - M_PI);
	}
	double zone_number_to_central_longitude(int zone_number)
	{
		return double((zone_number - 1) * 6 - 180 + 3);
	}
	string lation_to_zone_letter(double latitude)
	{
		if (latitude >= -80 && latitude <= 84)
			return "ZONE_LETTERS[int(latitude + 80) >> 3]";
		else
			return "";
	}
	int latlon_to_zone_number(double latitude, double longitude)
	{
		if (latitude >= 56 && latitude < 64 && longitude >= 3 && longitude < 12)
			return 32;
		if (latitude >= 72 && latitude <= 84 && longitude >= 0)
		{
			if (longitude < 9)
				return 31;
			else if (longitude < 21)
				return 33;
			else if (longitude < 33)
				return 35;
			else if (longitude < 42)
				return 37;
		}
		return int((longitude + 180) / 6) + 1;
	}
	void check_valid_zone(int force_zone_number, string force_zone_letter)
	{
		if (force_zone_number < 1 || force_zone_number>60)
		{
			throw "zone number out of range (must be between 1 and 60)";
		}
		if (!force_zone_letter.empty())
		{
			transform(force_zone_letter.begin(), force_zone_letter.end(), force_zone_letter.begin(), ::toupper);
			if (force_zone_letter < "C" || force_zone_letter>"X" || force_zone_letter == "I" || force_zone_letter == "O")
			{
				throw "zone letter out of range (must be between C and X)";
			}
		}
	}
	bool from_latlon(double latitude, double longitude, double& easting, double& norting, int& zone_number, string& zone_letter, int force_zone_number = 0, string force_zone_letter = "") {
		if (latitude < -80.0 || latitude>84.0)
		{
			//throw printf("latitude out of range (must be between 80 deg S and 84 deg N)");
			return 0;
		}
		else if (longitude < -180.0 || longitude>180.0) {
			//throw printf("longitude out of range (must be between 180 deg W and 180 deg E)");
			return 0;
		}
		else
		{
			if (force_zone_number != 0) {
				check_valid_zone(force_zone_number, force_zone_letter);
				zone_number = force_zone_number;
			}
			else
			{
				zone_number = latlon_to_zone_number(latitude, longitude);
			}
			if (force_zone_letter.empty())
			{
				zone_letter = lation_to_zone_letter(latitude);
			}
			else
			{
				zone_letter = force_zone_letter;
			}
			double lat_rad = latitude / Rad_to_deg;
			double lat_sin = sin(lat_rad);
			double lat_cos = cos(lat_rad);
			double lat_tan = lat_sin / lat_cos;
			double lat_tan2 = lat_tan * lat_tan;
			double lat_tan4 = lat_tan2 * lat_tan2;
			double lon_rad = longitude / Rad_to_deg;
			double central_lon = zone_number_to_central_longitude(zone_number);
			double central_lon_rad = central_lon / Rad_to_deg;
			double n = R / sqrt(1 - E * lat_sin * lat_sin);
			double c = E_P2 * lat_cos * lat_cos;
			double a = lat_cos * mod_angle(lon_rad - central_lon_rad);
			double a2 = a * a;
			double a3 = a2 * a;
			double a4 = a3 * a;
			double a5 = a4 * a;
			double a6 = a5 * a;
			double m = R * (M1 * lat_rad - M2 * sin(2 * lat_rad) + M3 * sin(4 * lat_rad) - M4 * sin(6 * lat_rad));
			easting = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000;
			norting = K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) + a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)));
			if (latitude < 0)
			{
				norting += 10000000;
			}
			return 1;
		}
	}
}



