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
}
