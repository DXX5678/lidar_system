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
}
