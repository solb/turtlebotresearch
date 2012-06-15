#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/method_types.h"

ros::Publisher outgoing;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr working(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ> out;
	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> downsample;
	pcl::SACSegmentation<pcl::PointXYZ> planes;
	pcl::PointIndices inliers;
	pcl::ModelCoefficients coefficients;

	crop.setInputCloud(in);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(0, 0.37);
	crop.filter(*working);

	downsample.setInputCloud(working);
	downsample.setLeafSize(0.04f, 0.04f, 0.04f);
	downsample.filter(*working);

	planes.setInputCloud(working);
	planes.setMethodType(pcl::SAC_RANSAC);
	planes.setModelType(pcl::SACMODEL_PLANE);
	planes.setDistanceThreshold(0.01);
	planes.setMaxIterations(1000);
	planes.segment(inliers, coefficients);

	for(std::vector<int>::iterator point=inliers.indices.begin(); point<inliers.indices.end(); point++)
		out.push_back((*working)[*point]);
		//ROS_INFO("%d", *point); //wtf?!
		//ROS_INFO("(%f,%f,%f)", (*working)[*point].x, (*working)[*point].y, (*working)[*point].z); //wth...
	//ROS_INFO("inliers: %d vs. points: %d so SAME: %d", (int)inliers.indices.size(), (int)out.points.size(), inliers.indices.size()==out.points.size());

	working->swap(out);
	outgoing.publish(*working);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	ros::NodeHandle node;

	ros::Subscriber incoming=node.subscribe("/cloud_throttled", 1, callback);
	outgoing=node.advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	ros::spin();
}
