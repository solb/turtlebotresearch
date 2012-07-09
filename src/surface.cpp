#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/surface/concave_hull.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle* node;
ros::Publisher visualizer;
ros::Publisher visualizer2;
ros::Publisher drive;
double last_FLOOR_CLOSEY=0, last_FLOOR_CLOSEZ=0, last_FLOOR_FARY=0, last_FLOOR_FARZ=0; //only recalculate the below when necessary
double FLOOR_SLOPE, FLOOR_YINTERCEPT; //model the floor's location
double steering=0; //current drive system angular command TODO

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	//these "constant" declarations may be partially generated from the read block below using the macro, but BEWARE OF TYPES:
	#if 0
	Jd2/\ui, f)d$
	#endif
	double DOWNSAMPLE_LEAFSIZE, CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, FLOOR_CLOSEY, FLOOR_CLOSEZ, FLOOR_FARY, FLOOR_FARZ, FLOOR_TOLERANCEFACTOR, HULL_ALPHA, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	bool DRIVE_MOVE, DISPLAY_TUNNELVISION, DISPLAY_DECISIONS;

	//read updated values for "constants" ... may be generated from declarations using the macro:
	#if 0
	^fsrg2f"F/l"vyt"f,wdt)"vPvb~f;d$a;j
	#endif
	node->getParam("/xbot_surface/downsample_leafsize", DOWNSAMPLE_LEAFSIZE);
	node->getParam("/xbot_surface/crop_xradius", CROP_XRADIUS);
	node->getParam("/xbot_surface/crop_ymin", CROP_YMIN);
	node->getParam("/xbot_surface/crop_ymax", CROP_YMAX);
	node->getParam("/xbot_surface/crop_zmin", CROP_ZMIN);
	node->getParam("/xbot_surface/crop_zmax", CROP_ZMAX);
	node->getParam("/xbot_surface/floor_closey", FLOOR_CLOSEY);
	node->getParam("/xbot_surface/floor_closez", FLOOR_CLOSEZ);
	node->getParam("/xbot_surface/floor_fary", FLOOR_FARY);
	node->getParam("/xbot_surface/floor_farz", FLOOR_FARZ);
	node->getParam("/xbot_surface/floor_tolerancefactor", FLOOR_TOLERANCEFACTOR);
	node->getParam("/xbot_surface/hull_alpha", HULL_ALPHA);
	node->getParam("/xbot_surface/drive_linearspeed", DRIVE_LINEARSPEED); //TODO
	node->getParam("/xbot_surface/drive_angularspeed", DRIVE_ANGULARSPEED); //TODO
	node->getParam("/xbot_surface/drive_move", DRIVE_MOVE); //TODO
	node->getParam("/xbot_surface/display_tunnelvision", DISPLAY_TUNNELVISION);
	node->getParam("/xbot_surface/display_decisions", DISPLAY_DECISIONS); //TODO

	//model the line of the floor iff the user changed its keypoints
	if(FLOOR_CLOSEY!=last_FLOOR_CLOSEY || FLOOR_CLOSEZ!=last_FLOOR_CLOSEZ || FLOOR_FARY!=last_FLOOR_FARY || FLOOR_FARZ!=last_FLOOR_FARZ)
	{
		FLOOR_SLOPE=(FLOOR_FARY-FLOOR_CLOSEY)/(FLOOR_FARZ-FLOOR_CLOSEZ);
		FLOOR_YINTERCEPT=(FLOOR_CLOSEY+FLOOR_FARY)/2-FLOOR_SLOPE*(FLOOR_CLOSEZ+FLOOR_FARZ)/2;
		last_FLOOR_CLOSEY=FLOOR_CLOSEY;
		last_FLOOR_FARY=FLOOR_FARY;
		last_FLOOR_CLOSEZ=FLOOR_CLOSEZ;
		last_FLOOR_FARZ=FLOOR_FARZ;
	}

	//variable declarations/initializations
	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> downsample;
	pcl::ConcaveHull<pcl::PointXYZ> footprint;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr front(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> hull;
	pcl::PointCloud<pcl::PointXYZ>::Ptr display(DISPLAY_TUNNELVISION ? front : out);
	geometry_msgs::Twist directions;

	//downsample cloud
	downsample.setInputCloud(in);
	downsample.setLeafSize((float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE);
	downsample.filter(*out);

	//create center "tunnel vision" region and store point count
	crop.setInputCloud(out);
	crop.setFilterFieldName("x");
	crop.setFilterLimits(-CROP_XRADIUS, CROP_XRADIUS);
	crop.filter(*front);

	crop.setInputCloud(front);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(CROP_YMIN, CROP_YMAX);
	crop.filter(*front);

	crop.setInputCloud(front);
	crop.setFilterFieldName("z");
	crop.setFilterLimits(CROP_ZMIN, CROP_ZMAX);
	crop.filter(*front);

	//ignore everything that is not the floor
	{
		pcl::PointCloud<pcl::PointXYZ>::iterator location=front->begin();
		while(location<front->end())
		{
			if(fabs(location->y/*point's actual y-coordinate*/ - (FLOOR_SLOPE*location->z+FLOOR_YINTERCEPT)/*floor's expected y-coordinate*/)>FLOOR_TOLERANCEFACTOR*DOWNSAMPLE_LEAFSIZE) //this point isn't part of the floor
				location=front->erase(location);
			else location++; //spare this floor particle
		}
	}

	//obtain a convex hull
	footprint.setInputCloud(front);
	footprint.setAlpha(HULL_ALPHA);
	footprint.reconstruct(hull);
	ROS_INFO("Hull contains %3d points", (int)hull.size());

	visualizer.publish(*display);
	visualizer2.publish(hull);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle;

	//declare constants
	node->setParam("/xbot_surface/downsample_leafsize", 0.01);
	node->setParam("/xbot_surface/crop_xradius", 0.25); //the robot's radius
	node->setParam("/xbot_surface/crop_ymin", 0.3); //should be lt [floor_closey,floor_fary]
	node->setParam("/xbot_surface/crop_ymax", 1.0); //should be gt [floor_closey,floor_fary]
	node->setParam("/xbot_surface/crop_zmin", 0.0); //should be lt [floor_closez,floor_farz]
	node->setParam("/xbot_surface/crop_zmax", 2); //recommended gte [floor_closez,floor_farz], but shorten if the data is very noisy
	node->setParam("/xbot_surface/floor_closey", 0.3625); //y-coordinate of floor's close boundary, used to approximate its slope
	node->setParam("/xbot_surface/floor_closez", 0.8); //z-coordinate corresponding to above, used to approximate its slope
	node->setParam("/xbot_surface/floor_fary", 0.47); //y-coordinate of floor's far boundary, used to approximate its slope
	node->setParam("/xbot_surface/floor_farz", 2.5); //z-coordinate of floor's far boundary, used to approximate its slope
	node->setParam("/xbot_surface/floor_tolerancefactor", 3); //automatically multiplied by DOWNSAMPLE_LEAFSIZE to yield maximum allowable y-coordinate devation of floor points
	node->setParam("/xbot_surface/hull_alpha", 0.1);
	node->setParam("/xbot_surface/drive_linearspeed", 0.3);
	node->setParam("/xbot_surface/drive_angularspeed", 0.4);
	node->setParam("/xbot_surface/drive_move", false); //set this to actually go somewhere!
	node->setParam("/xbot_surface/display_tunnelvision", false); //sends the straight-ahead, shortened view instead of long, panaramic one
	node->setParam("/xbot_surface/display_decisions", false); //limits point cloud output to still frames when the robot decides which way to go

	//request and pass messages
	ros::Subscriber incoming=node->subscribe("/cloud_throttled", 1, callback);
	visualizer=node->advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	visualizer2=node->advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_hull", 1);
	drive=node->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::spin();

	//clean up constants ... may be generated from declarations using the macro:
	#if 0
	:s/set/deletef,dt)f;d$a;j
	#endif
	node->deleteParam("/xbot_surface/downsample_leafsize");
	node->deleteParam("/xbot_surface/crop_xradius");
	node->deleteParam("/xbot_surface/crop_ymin");
	node->deleteParam("/xbot_surface/crop_ymax");
	node->deleteParam("/xbot_surface/crop_zmin");
	node->deleteParam("/xbot_surface/crop_zmax");
	node->deleteParam("/xbot_surface/floor_closey");
	node->deleteParam("/xbot_surface/floor_closez");
	node->deleteParam("/xbot_surface/floor_fary");
	node->deleteParam("/xbot_surface/floor_farz");
	node->deleteParam("/xbot_surface/floor_tolerancefactor");
	node->deleteParam("/xbot_surface/hull_alpha");
	node->deleteParam("/xbot_surface/drive_linearspeed");
	node->deleteParam("/xbot_surface/drive_angularspeed");
	node->deleteParam("/xbot_surface/drive_move");
	node->deleteParam("/xbot_surface/display_tunnelvision");
	node->deleteParam("/xbot_surface/display_decisions");

	delete node;
}
