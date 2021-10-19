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
	void writeLas_l(string file, list<Matrix<double, 7, Dynamic >> final)
	{
		string filename = file;
		Options xjOptions;
		xjOptions.add("filename", filename);

		list<Matrix<double, 7, Dynamic >>::iterator idk = final.begin();
		list<Matrix<double, 7, Dynamic >>::iterator idj = final.end();

		double xoffset = 0.000;
		double yoffset = 0.000;
		double zoffset = 0.000;
		double xmax = (*idk)(1, 0), xmin = (*idk)(1, 0);
		double ymax = (*idk)(2, 0), ymin = (*idk)(2, 0);
		double zmax = (*idk)(3, 0), zmin = (*idk)(3, 0);
		for (list<Matrix<double, 7, Dynamic >>::iterator id = idk; id != idj; id++)
		{
			xmax = max(xmax, (*id).row(1).maxCoeff());
			ymax = max(ymax, (*id).row(2).maxCoeff());
			zmax = max(zmax, (*id).row(3).maxCoeff());
			xmin = min(xmin, (*id).row(1).minCoeff());
			ymin = min(ymin, (*id).row(2).minCoeff());
			zmin = min(zmin, (*id).row(3).minCoeff());
		}
		if (ymin < 0 || xmin < 0)
		{
			//printf("UTM should not allow negative x and y");
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

		//long long int tj = 0;

		long long int num = 0;
		for (list<Matrix<double, 7, Dynamic >>::iterator id = idk; id != idj; id++)
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
				/*
				if (x < 0 || y < 0 || z < 0)
				{
					tj++;
				}
				*/
				int intensity = int((*id)(4, i));
				int wavenumber = int((*id)(5, i));
				view->setField(Dimension::Id::GpsTime, i + num, GPStime);
				view->setField(Dimension::Id::X, i + num, x);
				view->setField(Dimension::Id::Y, i + num, y);
				view->setField(Dimension::Id::Z, i + num, z);
				view->setField(Dimension::Id::Intensity, i + num, intensity);
				view->setField(Dimension::Id::NumberOfReturns, i + num, wavenumber);
			}
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

	void write_imu(string file, MatrixXd final)
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


		PointViewPtr view(new PointView(table));

		for (int i = 0; i < final.cols(); i++)
		{
			double GPStime = final(0, i);
			double x = final(1, i);
			double y = final(2, i);
			double z = final(3, i);
			view->setField(Dimension::Id::GpsTime, i, GPStime);
			view->setField(Dimension::Id::X, i, x);
			view->setField(Dimension::Id::Y, i, y);
			view->setField(Dimension::Id::Z, i, z);
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

	void AwriteLas_l(string file, list<Matrix<double, 6, Dynamic >> final)
	{
		string filename = file;
		Options xjOptions;
		xjOptions.add("filename", filename);

		list<Matrix<double, 6, Dynamic >>::iterator idk = final.begin();
		list<Matrix<double, 6, Dynamic >>::iterator idj = final.end();

		double xoffset = 0.000;
		double yoffset = 0.000;
		double zoffset = 0.000;
		double xmax = (*idk)(1, 0), xmin = (*idk)(1, 0);
		double ymax = (*idk)(2, 0), ymin = (*idk)(2, 0);
		double zmax = (*idk)(3, 0), zmin = (*idk)(3, 0);
		for (list<Matrix<double, 6, Dynamic >>::iterator id = idk; id != idj; id++)
		{
			xmax = max(xmax, (*id).row(1).maxCoeff());
			ymax = max(ymax, (*id).row(2).maxCoeff());
			zmax = max(zmax, (*id).row(3).maxCoeff());
			xmin = min(xmin, (*id).row(1).minCoeff());
			ymin = min(ymin, (*id).row(2).minCoeff());
			zmin = min(zmin, (*id).row(3).minCoeff());
		}
		if (ymin < 0 || xmin < 0)
		{
			//printf("UTM should not allow negative x and y");
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

		//long long int tj = 0;

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
				/*
				if (x < 0 || y < 0 || z < 0)
				{
					tj++;
				}
				*/
				int intensity = int((*id)(4, i));
				int wavenumber = int((*id)(5, i));
				view->setField(Dimension::Id::GpsTime, i + num, GPStime);
				view->setField(Dimension::Id::X, i + num, x);
				view->setField(Dimension::Id::Y, i + num, y);
				view->setField(Dimension::Id::Z, i + num, z);
				view->setField(Dimension::Id::Intensity, i + num, intensity);
				view->setField(Dimension::Id::NumberOfReturns, i + num, wavenumber);
			}
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
