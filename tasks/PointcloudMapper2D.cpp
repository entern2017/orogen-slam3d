#include "PointcloudMapper2D.hpp"

#include <envire/Core.hpp>
#include <envire/Orocos.hpp>

#include <pcl/common/transforms.h>

using namespace slam3d;

// https://de.wikipedia.org/wiki/Bresenham-Algorithmus#Kompakte_Variante
void line(int x0, int y0, int x1, int y1, int* array, int sizex, int sizey)
{
	int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
	int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
	int err = dx+dy, e2; /* error value e_xy */

	while(1)
	{
		if(x0 >= 0 && x0 < sizex && y0 >= 0 && y0 < sizey)
		{
			array[(y0 * sizex) + x0] += 1;
		}
		if (x0==x1 && y0==y1) break;
		e2 = 2*err;
		if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
		if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
	}
}

PointcloudMapper2D::PointcloudMapper2D(std::string const& name)
    : PointcloudMapper2DBase(name)
{
}

PointcloudMapper2D::PointcloudMapper2D(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapper2DBase(name, engine)
{
}

PointcloudMapper2D::~PointcloudMapper2D()
{
}

bool PointcloudMapper2D::generate_map()
{
	// Setup a temporary array
	size_t size_x = mGrid->getCellSizeX();
	size_t size_y = mGrid->getCellSizeY();
	size_t arr_size = size_x * size_y;
	int occ[arr_size];
	int hit[arr_size];
	memset(occ, 0, arr_size * sizeof(int));
	memset(hit, 0, arr_size * sizeof(int));	
	
	// Project pointclouds to temporary array
	mLogger->message(INFO, "Requested traversability-grid generation.");
	VertexList vertices = mMapper->getVerticesFromSensor(mPclSensor->getName());
	int count = 0;
	int valid = 0;
	for(VertexList::const_iterator it = vertices.begin(); it < vertices.end(); it++)
	{
		PointCloudMeasurement* pcl = dynamic_cast<PointCloudMeasurement*>((*it)->measurement);
		if(!pcl)
		{
			mLogger->message(ERROR, "Measurement from PCL-Sensor is not a point cloud!");
			return false;
		}
		
		// Get sensor's pose in map coordinates
		Transform sensorPose = (*it)->corrected_pose * pcl->getSensorPose();
		
		size_t sensorX, sensorY, pointX, pointY;
		if(mGrid->toGrid(sensorPose.translation(), sensorX, sensorY))
		{
			PointCloud::Ptr cloud(new PointCloud);
			pcl::transformPointCloud(*(pcl->getPointCloud()), *cloud, sensorPose.matrix());
			for(PointCloud::const_iterator it = cloud->begin(); it < cloud->end(); it++)
			{
				if(mGrid->toGrid(Eigen::Vector3d(it->x, it->y, 0), pointX, pointY))
				{
					occ[(pointY * size_x) + pointX] += 1;
					line(sensorX, sensorY, pointX, pointY, hit, size_x, size_y);
					valid++;
				}
				count++;
			}
		}
	}
	mLogger->message(DEBUG, (boost::format("Projected %1% out of %2% points to the grid.") % valid % count).str());
	
	// Write temporary array to traversability map
	size_t index = 0;
	for(size_t y = 0; y < size_y; y++)
	{
		for(size_t x = 0; x < size_x; x++, index++)
		{
			int k = 0;
			if(hit[index] > 0)
			{
				if((occ[index] / hit[index]) > 0.5)
				{
					k = 1;
				}else
				{
					k = 2;
				}
//				p = 2.0 * std::abs(0.5 - (occ[index] / hit[index]));
			}
			mGrid->setTraversabilityAndProbability(k, 1.0, x, y);
		}
	}
	
	
	envire::OrocosEmitter emitter(&mEnvironment, _envire_map);
//	emitter.setTime(mapTime);
	emitter.flush();
	return true;
}

bool PointcloudMapper2D::configureHook()
{
	if (! PointcloudMapper2DBase::configureHook())
		return false;
	
	// Read parameters
	mSizeX = _size_x.get();
	mSizeY = _size_y.get();
	mOffsetX = _offset_x.get();
	mOffsetY = _offset_y.get();
	mMinZ = _min_z.get();
	mMaxZ = _max_z.get();
	mResolution = _resolution.get();

	// Initialize envire stuff
	size_t x_size = mSizeX / mResolution;
	size_t y_size = mSizeY / mResolution;
	mGrid = new envire::TraversabilityGrid(x_size, y_size, mResolution, mResolution, mOffsetX, mOffsetY, "slam3d-grid");
    mGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.5));
	mGrid->setTraversabilityClass(1, envire::TraversabilityClass(0.0));
	mGrid->setTraversabilityClass(2, envire::TraversabilityClass(1.0));

	// Add grid to environment
	envire::FrameNode* grid_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), grid_node);
	mEnvironment.setFrameNode(mGrid, grid_node);
	return true;
}

bool PointcloudMapper2D::startHook()
{
	if (! PointcloudMapper2DBase::startHook())
		return false;
	return true;
}

void PointcloudMapper2D::updateHook()
{
	PointcloudMapper2DBase::updateHook();
}

void PointcloudMapper2D::errorHook()
{
	PointcloudMapper2DBase::errorHook();
}

void PointcloudMapper2D::stopHook()
{
	PointcloudMapper2DBase::stopHook();
}

void PointcloudMapper2D::cleanupHook()
{
	PointcloudMapper2DBase::cleanupHook();
}