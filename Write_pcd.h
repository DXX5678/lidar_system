#ifndef _Pcd_H
#define _Pcd_H
#endif
#include <Eigen/Core>
#include <list>
using namespace Eigen;
using namespace std;

namespace Pcd
{
	//生成pcd文件 矩阵形式
	void writePcd_m(string file, MatrixXd final);

	//生成pcd文件 列表形式
	void writePcd_l(string file, list<Matrix<double, 6, Dynamic >> final);
}
