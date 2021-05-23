#ifndef _Pcd_H
#define _Pcd_H
#endif
#include <Eigen/Core>
#include <list>
using namespace Eigen;
using namespace std;

namespace Pcd
{
	//����pcd�ļ� ������ʽ
	void writePcd_m(string file, MatrixXd final);

	//����pcd�ļ� �б���ʽ
	void writePcd_l(string file, list<Matrix<double, 6, Dynamic >> final);
}
