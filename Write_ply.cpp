#include "Write_ply.h"
#include <memory>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Stage.hpp>

#include <pdal/Filter.hpp>
#include <pdal/filters/DividerFilter.hpp>



#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>

#include <pdal/io/PlyReader.hpp>
#include <pdal/io/PlyWriter.hpp>

#include <pdal/io/TextReader.hpp>
#include <pdal/io/TextWriter.hpp>

#include <pdal/PluginDirectory.hpp>
#include <pdal/PluginManager.hpp>

using namespace std;
using namespace Eigen;

namespace Ply 
{
	void writePly_m(string file, MatrixXd final)
	{
		using namespace pdal;
		using namespace pdal::Dimension;

		string filename = file;
		Options xjOptions;
		xjOptions.add("filename", filename);
		xjOptions.add("storage_mode", "ascii");

		PointTable table;
		table.layout()->registerDim(Dimension::Id::GpsTime);
		table.layout()->registerDim(Dimension::Id::X);
		table.layout()->registerDim(Dimension::Id::Y);
		table.layout()->registerDim(Dimension::Id::Z);
		table.layout()->registerDim(Dimension::Id::Intensity);
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
		string w_drivername = factory.inferWriterDriver(filename);
		Stage* writer = factory.createStage(w_drivername);
		writer->addOptions(xjOptions);
		writer->setInput(xjBufferReader);
		writer->setOptions(xjOptions);
		writer->prepare(table);
		writer->execute(table);
	}

	void writePly_l(string file, list<Matrix<double, 6, Dynamic >> final)
	{
		using namespace pdal;
		using namespace pdal::Dimension;
		list<Matrix<double, 6, Dynamic >>::iterator idk = final.begin();
		list<Matrix<double, 6, Dynamic >>::iterator idj = final.end();

		string filename = file;
		Options xjOptions;
		xjOptions.add("filename", filename);
		xjOptions.add("storage_mode", "ascii");

		PointTable table;
		table.layout()->registerDim(Dimension::Id::GpsTime);
		table.layout()->registerDim(Dimension::Id::X);
		table.layout()->registerDim(Dimension::Id::Y);
		table.layout()->registerDim(Dimension::Id::Z);
		table.layout()->registerDim(Dimension::Id::Intensity);
		PointViewPtr view(new PointView(table));

		long long int num = 0;
		for (list<Matrix<double, 6, Dynamic >>::iterator id = idk; id != idj; id++)
		{
			if (id != idk)
			{
				id--;
				num += (*id).cols();
				id++;
			}
			for (int i = 0; i < (*id).cols(); i++)
			{
				double GPStime = (*id)(0, i);
				double x = (*id)(1, i);
				double y = (*id)(2, i);
				double z = (*id)(3, i);
				int intensity = int((*id)(4, i));
				view->setField(Dimension::Id::GpsTime, i + num, GPStime);
				view->setField(Dimension::Id::X, i + num, x);
				view->setField(Dimension::Id::Y, i + num, y);
				view->setField(Dimension::Id::Z, i + num, z);
				view->setField(Dimension::Id::Intensity, i + num, intensity);
			}
		}
		BufferReader xjBufferReader;
		xjBufferReader.addView(view);

		StageFactory factory;
		string w_drivername = factory.inferWriterDriver(filename);
		Stage* writer = factory.createStage(w_drivername);
		writer->addOptions(xjOptions);
		writer->setInput(xjBufferReader);
		writer->setOptions(xjOptions);
		writer->prepare(table);
		writer->execute(table);
	}
}
