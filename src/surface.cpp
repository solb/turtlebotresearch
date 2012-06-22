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

/**
Calculates the average depth of the points in the given cloud.
Precondition: There is at least 1 point in the cloud!
*/
float averageDepth(pcl::PointCloud<pcl::PointXYZ>& obstructions)
{
	float depths=0;

	for(std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator spot=obstructions.begin(); spot<obstructions.end(); spot++)
		depths+=spot->z;
	depths/=obstructions.size(); //average

	return depths;
}

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	//these declarations may be partially generated from the read block below using the macro, but BEWARE OF TYPES:
	#if 0
	Jd2/\ui, f)d$
	#endif
	double YCROP_MIN, YCROP_MAX, ZCROP_MIN, ZCROP_MAX, DOWNSAMPLE_LEAFSIZE, DRIVE_RADIUS, DRIVE_OBSTACLE, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	int DRIVE_SAMPLES;
	bool DRIVE_MOVE, DISPLAY_TUNNELVISION, DISPLAY_DECISIONS;

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
	node->getParam("/xbot_surface/display_tunnelvision", DISPLAY_TUNNELVISION);
	node->getParam("/xbot_surface/display_decisions", DISPLAY_DECISIONS);

	//variable declarations/initializations
	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr front(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr display=DISPLAY_TUNNELVISION ? front : out;
	geometry_msgs::Twist directions;
	int averageObstacles=0;

	//crop the cloud
	crop.setInputCloud(in);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(YCROP_MIN, YCROP_MAX);
	crop.filter(*out);

	//downsample cloud
	filter.setInputCloud(out);
	filter.setLeafSize((float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE);
	filter.filter(*out);

	//create center third and store point count
	crop.setInputCloud(out);
	crop.setFilterFieldName("x");
	crop.setFilterLimits(-DRIVE_RADIUS, DRIVE_RADIUS);
	crop.filter(*front);

	crop.setInputCloud(front);
	crop.setFilterFieldName("z");
	crop.setFilterLimits(ZCROP_MIN, ZCROP_MAX);
	crop.filter(*front);

	if(steering!=0) frontSamples.clear(); //use straight snapshots while turning
	frontSamples.push_front(front->size());
	while((int)frontSamples.size()>DRIVE_SAMPLES) frontSamples.pop_back(); //constrain our backlog

	//compute average number of points
	for(std::list<int>::iterator location=frontSamples.begin(); location!=frontSamples.end(); location++)
		averageObstacles+=*location;
	averageObstacles/=frontSamples.size();

	//let's DRIVE!
	ROS_INFO("Points in our way: %d", averageObstacles);
	if(averageObstacles<DRIVE_OBSTACLE) //there's "nothing" in our way
	{
		steering=0; //go straight on
		directions.linear.x=DRIVE_LINEARSPEED; //forward
	}
	else if(steering==0) //we were going straight, but now we need to turn (if we're still turning, we'll keep going the same direction to prevent oscillation)
	{
		pcl::PointCloud<pcl::PointXYZ> left, right;

		crop.setInputCloud(out);
		crop.setFilterFieldName("x");
		crop.setFilterLimits(-1, 0); //take left field of view
		crop.filter(left);

		crop.setInputCloud(out);
		crop.setFilterFieldName("x"); //take right field of view
		crop.setFilterLimits(0, 1);
		crop.filter(right);

		if(right.size()>0 && (left.size()==0 || averageDepth(left)>=averageDepth(right))) //left looks better
		{
			ROS_INFO(" ... moving %s", "LEFT");
			steering=DRIVE_ANGULARSPEED; //left
		}
		else //right it is
		{
			ROS_INFO(" ... moving %s", "RIGHT");
			steering=-DRIVE_ANGULARSPEED; //right
		}

		if(DISPLAY_DECISIONS) outgoing.publish(*display); //just made a steering decision
	}
	directions.angular.z=steering; //keep turning ... or not
	if(DRIVE_MOVE) drive.publish(directions);

	if(!DISPLAY_DECISIONS) outgoing.publish(*display); //publish to RViz constantly
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle;

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
	node->setParam("/xbot_surface/display_tunnelvision", false); //sends the straight-ahead, shortened view instead of long, panaramic one
	node->setParam("/xbot_surface/display_decisions", false); //limits point cloud output to still frames when the robot decides which way to go

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
	node->deleteParam("/xbot_surface/display_tunnelvision");
	node->deleteParam("/xbot_surface/display_decisions");

	delete node;
}
