#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle* node;
ros::Publisher outgoing;
ros::Publisher drive;
std::list<int> frontSamples; //last few samples of what's ahead
double steering=0; //current drive system angular command

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	//these declarations may be partially generated from the read block below using the macro, but BEWARE OF TYPES:
	#if 0
	d2/\uf)d$a, J
	#endif
	double YCROP_MIN, YCROP_MAX, ZCROP_MIN, ZCROP_MAX, DOWNSAMPLE_LEAFSIZE, DRIVE_RADIUS, DRIVE_OBSTACLE, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	int DRIVE_SAMPLES;
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
	node->getParam("/xbot_surface/drive_samples", DRIVE_SAMPLES);
	node->getParam("/xbot_surface/drive_obstacle", DRIVE_OBSTACLE);
	node->getParam("/xbot_surface/drive_linearspeed", DRIVE_LINEARSPEED);
	node->getParam("/xbot_surface/drive_angularspeed", DRIVE_ANGULARSPEED);
	node->getParam("/xbot_surface/drive_move", DRIVE_MOVE);

	//variable declarations/initializations
	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ> front;
	geometry_msgs::Twist directions;
	int averageObstacles=0;

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

	//create center third and store point count
	crop.setInputCloud(out);
	crop.setFilterFieldName("x");
	crop.setFilterLimits(-DRIVE_RADIUS, DRIVE_RADIUS);
	crop.filter(front);
	if(steering!=0) frontSamples.clear(); //use straight snapshots while turning
	frontSamples.push_front(front.size());
	while((int)frontSamples.size()>DRIVE_SAMPLES) frontSamples.pop_back(); //constrain our backlog

	//compute average number of points
	for(std::list<int>::iterator location=frontSamples.begin(); location!=frontSamples.end(); location++)
		averageObstacles+=*location;
	averageObstacles/=frontSamples.size();

	//let's DRIVE!
	if(averageObstacles<DRIVE_OBSTACLE) //there's "nothing" in our way
	{
		ROS_INFO("Trivial stuff detected: %d", averageObstacles);
		steering=0; //go straight on
		directions.linear.x=DRIVE_LINEARSPEED; //forward
	}
	else //evasive action required
	{
		pcl::PointCloud<pcl::PointXYZ> left, right;

		ROS_INFO("Too much DANGER ahead: %d", averageObstacles);
		
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
			ROS_INFO(" ... moving %s", "LEFT");
			steering=DRIVE_ANGULARSPEED; //left
		}
		else if(steering<=0) //not already going left
		{
			ROS_INFO(" ... moving %s", "RIGHT");
			steering=-DRIVE_ANGULARSPEED; //right
		}
		directions.angular.z=steering; //else: keep going the same way we were
	}
	if(DRIVE_MOVE) drive.publish(directions);

	outgoing.publish(*out); //show RViz our front and peripheral vision
	//outgoing.publish(front); //use instead for tunnel vision
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle();

	//declare constants
	node->setParam("/xbot_surface/ycrop_min", 0.0);
	node->setParam("/xbot_surface/ycrop_max", 0.34);
	node->setParam("/xbot_surface/zcrop_min", 0.0);
	node->setParam("/xbot_surface/zcrop_max", 1.25);
	node->setParam("/xbot_surface/downsample_leafsize", 0.03);
	node->setParam("/xbot_surface/drive_radius", 0.25); //lateral radius from center of boundaries between navigational thirds
	node->setParam("/xbot_surface/drive_samples", 5); //number of sensor readings to average in order to filter out noise (for front region only)
	node->setParam("/xbot_surface/drive_obstacle", 1); //minimum number of points that are considered an obstacle to our forward motion
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
	node->deleteParam("/xbot_surface/drive_samples");
	node->deleteParam("/xbot_surface/drive_samples");
	node->deleteParam("/xbot_surface/drive_obstacle");
	node->deleteParam("/xbot_surface/drive_linearspeed");
	node->deleteParam("/xbot_surface/drive_angularspeed");
	node->deleteParam("/xbot_surface/drive_move");

	delete node;
}
