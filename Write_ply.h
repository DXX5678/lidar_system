#ifndef _Ply_H
#define _Ply_H
#endif
#include <Eigen/Core>
using namespace Eigen;
using namespace std;
namespace Ply
{
	//生成ply文件
	void writePly_m(string file, MatrixXd final);
}
