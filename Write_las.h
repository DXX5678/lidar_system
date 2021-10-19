#ifndef _Las_H
#define _Las_H
#endif
#include <Eigen/Core>
#include <list>
using namespace std;
using namespace Eigen;

namespace Las
{
	//����las�ļ� �б���ʽ
	void writeLas_l(string file, list<Matrix<double, 7, Dynamic >> final);

	//����las�ļ� �б���ʽ-avia
	void AwriteLas_l(string file, list<Matrix<double, 6, Dynamic >> final);

	//����imu·��
	void write_imu(string file, MatrixXd final);
}
