#ifndef _Las_H
#define _Las_H
#endif
#include <Eigen/Core>
#include <list>
using namespace std;
using namespace Eigen;

namespace Las
{
	//����las�ļ� ������ʽ
	void writeLas_m(string file, MatrixXd final);

	//����las�ļ� �б���ʽ
	void writeLas_l(string file, list<Matrix<double, 6, Dynamic >> final);
}
