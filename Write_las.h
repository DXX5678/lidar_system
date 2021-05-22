#ifndef _Las_H
#define _Las_H
#endif
#include <Eigen/Core>
using namespace std;
using namespace Eigen;

namespace Las
{
	//生成las文件
	void writeLas_m(string file, MatrixXd final);
}
