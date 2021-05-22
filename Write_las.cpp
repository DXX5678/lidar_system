#include <memory>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Filter.hpp>
#include <pdal/filters/SMRFilter.hpp>
#include <pdal/filters/RangeFilter.hpp>

#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/LasHeader.hpp>


#include "Write_las.h"

using namespace std;
using namespace Eigen;
using namespace pdal;

namespace Las
{
	void writeLas_m(string file, MatrixXd final)
	{
		string filename = file;
		Options xjOptions;
		xjOptions.add("filename", filename);

		double xoffset = 0.000;
		double yoffset = 0.000;
		double zoffset = 0.000;
		double xmax = final.row(1).maxCoeff(), xmin = final.row(1).minCoeff();
		double ymax = final.row(2).maxCoeff(), ymin = final.row(2).minCoeff();
		double zmax = final.row(3).maxCoeff(), zmin = final.row(3).minCoeff();

		if (ymin < 0 || xmin < 0)
		{
			//printf("UTM should not allow negative x and y\n");
		}
		if (xmax > 1000)
		{
			xoffset = double(int(xmin));
		}
		if (ymax > 1000)
		{
			yoffset = double(int(ymin));
		}
		if (zmax > 1000)
		{
			zoffset = double(int(zmin));
		}
		xjOptions.add("offset_x", xoffset);
		xjOptions.add("offset_y", yoffset);
		xjOptions.add("offset_z", zoffset);
		xjOptions.add("scale_x", 0.001);
		xjOptions.add("scale_y", 0.001);
		xjOptions.add("scale_z", 0.001);

		PointTable table;
		table.layout()->registerDim(Dimension::Id::GpsTime);
		table.layout()->registerDim(Dimension::Id::X);
		table.layout()->registerDim(Dimension::Id::Y);
		table.layout()->registerDim(Dimension::Id::Z);
		table.layout()->registerDim(Dimension::Id::Intensity);
		table.layout()->registerDim(Dimension::Id::NumberOfReturns);

		PointViewPtr view(new PointView(table));

		for (int i = 0; i < final.cols(); i++)
		{
			double GPStime = final(0, i);
			double x = final(1, i);
			double y = final(2, i);
			double z = final(3, i);
			int intensity = int(final(4, i));
			int wavenumber = int(final(5, i));
			view->setField(Dimension::Id::GpsTime, i, GPStime);
			view->setField(Dimension::Id::X, i, x);
			view->setField(Dimension::Id::Y, i, y);
			view->setField(Dimension::Id::Z, i, z);
			view->setField(Dimension::Id::Intensity, i, intensity);
			view->setField(Dimension::Id::NumberOfReturns, i, wavenumber);
		}

		BufferReader xjBufferReader;
		xjBufferReader.addView(view);

		StageFactory factory;
		Stage* writer = factory.createStage("writers.las");
		writer->setInput(xjBufferReader);
		writer->setOptions(xjOptions);
		writer->prepare(table);
		writer->execute(table);
	}
}
