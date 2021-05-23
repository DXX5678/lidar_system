#ifndef _Ply_H
#define _Ply_H
#endif
#include <Eigen/Core>
#include <list>
using namespace Eigen;
using namespace std;

namespace Ply
{
	//生成ply文件 矩阵形式
	void writePly_m(string file, MatrixXd final);

	//生成ply文件 列表形式
	void writePly_l(string file, list<Matrix<double, 6, Dynamic >> final);
}
