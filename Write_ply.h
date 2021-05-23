#ifndef _Ply_H
#define _Ply_H
#endif
#include <Eigen/Core>
#include <list>
using namespace Eigen;
using namespace std;

namespace Ply
{
	//����ply�ļ� ������ʽ
	void writePly_m(string file, MatrixXd final);

	//����ply�ļ� �б���ʽ
	void writePly_l(string file, list<Matrix<double, 6, Dynamic >> final);
}
