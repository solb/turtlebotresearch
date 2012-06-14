#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

ros::Publisher outgoing;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	outgoing.publish(*in);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	ros::NodeHandle node;

	ros::Subscriber incoming=node.subscribe("/cloud_throttled", 1, callback);
	outgoing=node.advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	ros::spin();
}
