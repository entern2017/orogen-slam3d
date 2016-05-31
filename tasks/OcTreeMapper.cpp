#include "OcTreeMapper.hpp"

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <pcl/common/transforms.h>

#include <envire/Orocos.hpp>

using namespace slam3d;

OcTreeMapper::OcTreeMapper(std::string const& name)
    : OcTreeMapperBase(name)
{
}

OcTreeMapper::OcTreeMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : OcTreeMapperBase(name, engine)
{
}

OcTreeMapper::~OcTreeMapper()
{
}

bool OcTreeMapper::generate_map()
{
	// Reset OctoMap
	delete mOcTree;
	mOcTree = new octomap::OcTree(mMapResolution);

	// Project all scans to octomap
	mLogger->message(INFO, "Requested octomap generation.");
	VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
	boost::thread projThread(&OcTreeMapper::buildOcTree, this, vertices);
	return true;
}

void OcTreeMapper::addScanToOctoMap(const VertexObject& scan)
{
	PointCloudMeasurement::Ptr pcl = boost::dynamic_pointer_cast<PointCloudMeasurement>(scan.measurement);
	if(!pcl)
	{
		mLogger->message(ERROR, "Measurement is not a point cloud!");
		throw BadMeasurementType();
	}

	PointCloud::Ptr tempCloud(new PointCloud);
	pcl::transformPointCloud(*(pcl->getPointCloud()), *tempCloud, (scan.corrected_pose * pcl->getSensorPose()).matrix());

	octomap::Pointcloud octoCloud;
	for(PointCloud::iterator it = tempCloud->begin(); it < tempCloud->end(); ++it)
	{
		octoCloud.push_back(octomap::point3d(it->x, it->y,it->z));
	}
	Vector3 origin = scan.corrected_pose.translation();
	mOcTree->insertPointCloud(octoCloud, octomap::point3d(origin(0), origin(1), origin(2)), -1, true, true);
}

void OcTreeMapper::buildOcTree(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	try
	{
		boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
		for(VertexObjectList::const_iterator it = vertices.begin(); it != vertices.end(); it++)
		{
			addScanToOctoMap(*it);
		}
	}catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build OcTree!");
		return;
	}
	mOcTree->updateInnerOccupancy();
	mOcTree->writeBinary("slam3d_octomap.bt");
	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated OcTree from %1% scans in %2% seconds.") % vertices.size() % duration).str());
	
	// Publish the MLS-Map	
	buildMLS();
	envire::OrocosEmitter emitter(&mEnvironment, _envire_map);
	emitter.flush();
}

void OcTreeMapper::buildMLS()
{
	mMultiLayerMap->clear();
	octomap::OcTree::leaf_iterator leaf = mOcTree->begin_leafs();
	octomap::OcTree::leaf_iterator end = mOcTree->end_leafs();
	
	for( ; leaf != end; ++leaf)
	{
		if(!mOcTree->isNodeOccupied(*leaf))
		{
			continue;
		}
		double half_size = leaf.getSize() / 2.0;
		double x_min = leaf.getX() - half_size;
		double y_min = leaf.getY() - half_size;
		double x_max = leaf.getX() + half_size;
		double y_max = leaf.getY() + half_size;
		
		for(double y = y_min; y <= y_max; y += mGridResolution)
		{
			for(double x = x_min; x <= x_max; x += mGridResolution)
			{
				envire::SurfacePatch patch(leaf.getZ() + half_size, 0, leaf.getSize(), envire::SurfacePatch::HORIZONTAL);
				mMultiLayerMap->update(Eigen::Vector2d(x, y) , patch);
			}
		}
	}
}

bool OcTreeMapper::configureHook()
{
	if (! OcTreeMapperBase::configureHook())
		return false;
		
	mOcTree = new octomap::OcTree(mGridResolution);
	return true;
}

bool OcTreeMapper::startHook()
{
	if (! OcTreeMapperBase::startHook())
		return false;
	return true;
}

void OcTreeMapper::updateHook()
{
	OcTreeMapperBase::updateHook();
}

void OcTreeMapper::errorHook()
{
	OcTreeMapperBase::errorHook();
}

void OcTreeMapper::stopHook()
{
	OcTreeMapperBase::stopHook();
}

void OcTreeMapper::cleanupHook()
{
	OcTreeMapperBase::cleanupHook();
	delete mOcTree;
}
