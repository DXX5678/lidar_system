#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "Write_pcd.h"

using namespace std;
using namespace Eigen;

namespace Pcd 
{
	void writePcd_m(string file, MatrixXd final)
	{

		long long int count = 0;
		count = final.cols();
		string filename = file;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.clear();
		for (int i = 0; i < final.cols(); i++)
		{
			double x = final(1, i);
			double y = final(2, i);
			double z = final(3, i);
			pcl::PointXYZ thePt(x, y, z);
			cloud.push_back(thePt);
		}
		cloud.width = count;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(static_cast<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>::size_type>(cloud.width) * cloud.height);
		pcl::io::savePCDFileASCII(filename, cloud);
		cloud.clear();
	}

	void writePcd_l(string file, list<Matrix<double, 6, Dynamic >> final)
	{
		list<Matrix<double, 6, Dynamic >>::iterator idk = final.begin();
		list<Matrix<double, 6, Dynamic >>::iterator idj = final.end();
		long long int count = 0;
		for (list<Matrix<double, 6, Dynamic >>::iterator id = idk; id != idj; id++)
		{
			count += (*id).cols();
		}
		string filename = file;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.clear();
		for (list<Matrix<double, 6, Dynamic >>::iterator id = idk; id != idj; id++)
		{
			for (int i = 0; i < (*id).cols(); i++)
			{
				double x = (*id)(1, i);
				double y = (*id)(2, i);
				double z = (*id)(3, i);
				pcl::PointXYZ thePt(x, y, z);
				cloud.push_back(thePt);
			}
		}
		cloud.width = count;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(static_cast<std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>::size_type>(cloud.width) * cloud.height);
		pcl::io::savePCDFileASCII(filename, cloud);
		cloud.clear();
	}
}
