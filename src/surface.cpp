#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle* node;
ros::Publisher outgoing;
ros::Publisher drive;
double steering=0;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	//these declarations may be partially generated from the read block below using the macro, but BEWARE OF TYPES:
	#if 0
	d2/\uf)d$a, J
	#endif
	double YCROP_MIN, YCROP_MAX, ZCROP_MIN, ZCROP_MAX, DOWNSAMPLE_LEAFSIZE, DRIVE_RADIUS, DRIVE_OBSTACLE, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	bool DRIVE_MOVE;

	//read updated values for constants ... may be generated from declarations using the macro:
	#if 0
	^fsrg2f"F/l"vyt"f,wdt)"vPvb~f;d$a;j
	#endif
	node->getParam("/xbot_surface/ycrop_min", YCROP_MIN);
	node->getParam("/xbot_surface/ycrop_max", YCROP_MAX);
	node->getParam("/xbot_surface/zcrop_min", ZCROP_MIN);
	node->getParam("/xbot_surface/zcrop_max", ZCROP_MAX);
	node->getParam("/xbot_surface/downsample_leafsize", DOWNSAMPLE_LEAFSIZE);
	node->getParam("/xbot_surface/drive_radius", DRIVE_RADIUS);
	node->getParam("/xbot_surface/drive_obstacle", DRIVE_OBSTACLE);
	node->getParam("/xbot_surface/drive_linearspeed", DRIVE_LINEARSPEED);
	node->getParam("/xbot_surface/drive_angularspeed", DRIVE_ANGULARSPEED);
	node->getParam("/xbot_surface/drive_move", DRIVE_MOVE);

	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());

	//crop the cloud
	crop.setInputCloud(in);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(YCROP_MIN, YCROP_MAX);
	crop.filter(*out);

	crop.setInputCloud(out);
	crop.setFilterFieldName("z");
	crop.setFilterLimits(ZCROP_MIN, ZCROP_MAX);
	crop.filter(*out);

	//downsample cloud
	filter.setInputCloud(out);
	filter.setLeafSize((float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE);
	filter.filter(*out);

	//let's DRIVE!
	pcl::PointCloud<pcl::PointXYZ> front;
	geometry_msgs::Twist directions;

	//create center third
	crop.setInputCloud(out);
	crop.setFilterFieldName("x");
	crop.setFilterLimits(-DRIVE_RADIUS, DRIVE_RADIUS);
	crop.filter(front);

	if(front.size()<DRIVE_OBSTACLE)
	{
		steering=0;
		directions.linear.x=DRIVE_LINEARSPEED; //forward
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ> left, right;

		ROS_INFO("Too much danger ahead: %d", (int)front.size());
		
		crop.setInputCloud(out);
		crop.setFilterFieldName("x");
		crop.setFilterLimits(-1, -DRIVE_RADIUS);
		crop.filter(left);

		crop.setInputCloud(out);
		crop.setFilterFieldName("x");
		crop.setFilterLimits(DRIVE_RADIUS, 1);
		crop.filter(right);

		if(left.size()<right.size() && steering>=0) //left looks better and we're not already going right (this would permit oscillation)
		{
			ROS_INFO(" ... moving %s\n", "LEFT");
			steering=DRIVE_ANGULARSPEED; //left
		}
		else if(steering<=0) //not already going left
		{
			ROS_INFO(" ... moving %s\n", "RIGHT");
			steering=-DRIVE_ANGULARSPEED; //right
		}
		directions.angular.z=steering; //else: keep going the same way
	}
	if(DRIVE_MOVE) drive.publish(directions);

	outgoing.publish(*out);
	//outgoing.publish(front);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle();

	//declare constants
	node->setParam("/xbot_surface/ycrop_min", 0.0);
	node->setParam("/xbot_surface/ycrop_max", 0.36);
	node->setParam("/xbot_surface/zcrop_min", 0.0);
	node->setParam("/xbot_surface/zcrop_max", 2.0);
	node->setParam("/xbot_surface/downsample_leafsize", 0.03);
	node->setParam("/xbot_surface/drive_radius", 0.25); //lateral radius from center of boundaries between navigational thirds
	node->setParam("/xbot_surface/drive_obstacle", 25); //number of points that are considered an obstacle to our forward motion
	node->setParam("/xbot_surface/drive_linearspeed", 0.3);
	node->setParam("/xbot_surface/drive_angularspeed", 0.4);
	node->setParam("/xbot_surface/drive_move", false); //whether or not to actually move

	//request and pass messages
	ros::Subscriber incoming=node->subscribe("/cloud_throttled", 1, callback);
	outgoing=node->advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	drive=node->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::spin();

	//clean up constants ... may be generated from declarations using the macro:
	#if 0
	:s/set/deletef,dt)f;d$a;j
	#endif
	node->deleteParam("/xbot_surface/ycrop_min");
	node->deleteParam("/xbot_surface/ycrop_max");
	node->deleteParam("/xbot_surface/zcrop_min");
	node->deleteParam("/xbot_surface/zcrop_max");
	node->deleteParam("/xbot_surface/downsample_leafsize");
	node->deleteParam("/xbot_surface/drive_radius");
	node->deleteParam("/xbot_surface/drive_obstacle");
	node->deleteParam("/xbot_surface/drive_linearspeed");
	node->deleteParam("/xbot_surface/drive_angularspeed");
	node->deleteParam("/xbot_surface/drive_move");

	delete node;
}
