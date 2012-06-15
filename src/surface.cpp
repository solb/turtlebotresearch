#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"

ros::Publisher outgoing;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());

	crop.setInputCloud(in);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(0, 0.37);
	crop.filter(*out);

	filter.setInputCloud(out);
	filter.setLeafSize(0.04f, 0.04f, 0.04f);
	filter.filter(*out);

	outgoing.publish(*out);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	ros::NodeHandle node;

	ros::Subscriber incoming=node.subscribe("/cloud_throttled", 1, callback);
	outgoing=node.advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	ros::spin();
}
