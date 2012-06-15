#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"

ros::NodeHandle* node;
ros::Publisher outgoing;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	std::string PASSTHROUGH_DIMENSION;
	double PASSTHROUGH_MIN, PASSTHROUGH_MAX, VOXELGRID_LEAFSIZE;

	//read updated values for constants
	node->getParam("/xbot_surface/passthrough_dimension", PASSTHROUGH_DIMENSION);
	node->getParam("/xbot_surface/passthrough_min", PASSTHROUGH_MIN);
	node->getParam("/xbot_surface/passthrough_max", PASSTHROUGH_MAX);
	node->getParam("/xbot_surface/voxelgrid_leafsize", VOXELGRID_LEAFSIZE);

	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());

	//crop the cloud
	crop.setInputCloud(in);
	crop.setFilterFieldName(PASSTHROUGH_DIMENSION);
	crop.setFilterLimits(PASSTHROUGH_MIN, PASSTHROUGH_MAX);
	crop.filter(*out);

	//downsample cloud
	filter.setInputCloud(out);
	filter.setLeafSize((float)VOXELGRID_LEAFSIZE, (float)VOXELGRID_LEAFSIZE, (float)VOXELGRID_LEAFSIZE);
	filter.filter(*out);

	outgoing.publish(*out);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle();

	//declare constants
	node->setParam("/xbot_surface/passthrough_dimension", "y");
	node->setParam("/xbot_surface/passthrough_min", 0.0);
	node->setParam("/xbot_surface/passthrough_max", 0.36);
	node->setParam("/xbot_surface/voxelgrid_leafsize", 0.03);

	//request and pass messages
	ros::Subscriber incoming=node->subscribe("/cloud_throttled", 1, callback);
	outgoing=node->advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	ros::spin();

	//clean up constants
	node->deleteParam("/xbot_surface/passthrough_dimension");
	node->deleteParam("/xbot_surface/passthrough_min");
	node->deleteParam("/xbot_surface/passthrough_max");
	node->deleteParam("/xbot_surface/voxelgrid_leafsize");

	delete node;
}
