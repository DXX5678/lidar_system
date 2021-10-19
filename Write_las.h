#ifndef _Las_H
#define _Las_H
#endif
#include <Eigen/Core>
#include <list>
using namespace std;
using namespace Eigen;

namespace Las
{
	//生成las文件 列表形式
	void writeLas_l(string file, list<Matrix<double, 7, Dynamic >> final);

	//生成las文件 列表形式-avia
	void AwriteLas_l(string file, list<Matrix<double, 6, Dynamic >> final);

	//绘制imu路径
	void write_imu(string file, MatrixXd final);
}
