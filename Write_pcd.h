#ifndef _Pcd_H
#define _Pcd_H
#endif
#include <Eigen/Core>
using namespace Eigen;
using namespace std;
namespace Pcd
{
	//生成pcd文件
	void writePcd_m(string file, MatrixXd final);
}
