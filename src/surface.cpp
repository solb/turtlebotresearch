#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/features/integral_image_normal.h"
#include "pcl/kdtree/organized_data.h"
#include "pcl/features/boundary.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle* node;
ros::Publisher visualizer;
ros::Publisher visualizer2;
ros::Publisher drive;
double last_FLOOR_CLOSEY=0, last_FLOOR_CLOSEZ=0, last_FLOOR_FARY=0, last_FLOOR_FARZ=0; //only recalculate the below when necessary
double FLOOR_SLOPE, FLOOR_YINTERCEPT; //model the floor's location
double steering=0; //current drive system angular command TODO re-integrate

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	//these "constant" declarations may be partially generated from the read block below using the following two macros, but be careful of types:
	#if 0
	Jd2/\ui, f)d$
	^ExA;j
	#endif
	double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, FLOOR_CLOSEY, FLOOR_CLOSEZ, FLOOR_FARY, FLOOR_FARZ, FLOOR_TOLERANCE, EDGES_SEARCHRADIUS, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	bool FLOOR_TRANSFORM, DRIVE_MOVE, DISPLAY_DECISIONS;

	//read updated values for "constants" ... may be generated from declarations using the macro:
	#if 0
	^fsrg2f"F/l"vyt"f,wdt)"vPvb~f;d$a;j
	#endif
	node->getParam("/xbot_surface/crop_xradius", CROP_XRADIUS);
	node->getParam("/xbot_surface/crop_ymin", CROP_YMIN);
	node->getParam("/xbot_surface/crop_ymax", CROP_YMAX);
	node->getParam("/xbot_surface/crop_zmin", CROP_ZMIN);
	node->getParam("/xbot_surface/crop_zmax", CROP_ZMAX);
	node->getParam("/xbot_surface/floor_closey", FLOOR_CLOSEY);
	node->getParam("/xbot_surface/floor_closez", FLOOR_CLOSEZ);
	node->getParam("/xbot_surface/floor_fary", FLOOR_FARY);
	node->getParam("/xbot_surface/floor_farz", FLOOR_FARZ);
	node->getParam("/xbot_surface/floor_tolerance", FLOOR_TOLERANCE);
	node->getParam("/xbot_surface/edges_searchradius", EDGES_SEARCHRADIUS);
	node->getParam("/xbot_surface/drive_linearspeed", DRIVE_LINEARSPEED);
	node->getParam("/xbot_surface/drive_angularspeed", DRIVE_ANGULARSPEED);
	node->getParam("/xbot_surface/floor_transform", FLOOR_TRANSFORM);
	node->getParam("/xbot_surface/drive_move", DRIVE_MOVE);
	node->getParam("/xbot_surface/display_decisions", DISPLAY_DECISIONS);

	//model the plane of the floor iff the user changed its keypoints
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
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalize;
	pcl::OrganizedDataIndex<pcl::PointXYZ>::Ptr index(new pcl::OrganizedDataIndex<pcl::PointXYZ>);
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> edgeDetect;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::PointCloud<pcl::PointXYZ> boundaryPoints;
	geometry_msgs::Twist directions;

	//crop to focus exclusively on the approximate range of floor points
	crop.setInputCloud(in);
	crop.setFilterFieldName("x");
	crop.setFilterLimits(-CROP_XRADIUS, CROP_XRADIUS);
	crop.setKeepOrganized(true);
	crop.filter(*points);

	crop.setInputCloud(points);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(CROP_YMIN, CROP_YMAX);
	crop.setKeepOrganized(true);
	crop.filter(*points);

	crop.setInputCloud(points);
	crop.setFilterFieldName("z");
	crop.setFilterLimits(CROP_ZMIN, CROP_ZMAX);
	crop.setKeepOrganized(true);
	crop.filter(*points);

	//ignore everything that is not the floor
	for(pcl::PointCloud<pcl::PointXYZ>::iterator location=points->begin(); location<points->end(); location++)
	{
		if(fabs(location->y/*point's actual y-coordinate*/ - (FLOOR_SLOPE*location->z+FLOOR_YINTERCEPT)/*floor's expected y-coordinate*/)>FLOOR_TOLERANCE) //this point isn't part of the floor
		{ //these aren't the points we're looking for
			location->x=std::numeric_limits<float>::quiet_NaN();
			location->y=std::numeric_limits<float>::quiet_NaN();
			location->z=std::numeric_limits<float>::quiet_NaN();
		}
		else if(FLOOR_TRANSFORM) //and it is part of the floor
		{
			location->z=location->z+FLOOR_SLOPE/location->y; //transform the floor onto the XZ plane
			location->y=0; //flatten it
		}
	}

	//compute surface normals
	normalize.setInputCloud(points);
	normalize.setNormalEstimationMethod(normalize.AVERAGE_DEPTH_CHANGE);
	//normalize.setNormalEstimationMethod(normalize.AVERAGE_3D_GRADIENT); TODO recheck
	//normalize.setMaxDepthChangeFactor(0.02f); TODO recheck
	//normalize.setNormalSmoothingSize(10.0f); TODO recheck
	normalize.compute(*normals);

	//detect edge points
	edgeDetect.setInputCloud(points);
	edgeDetect.setInputNormals(normals);
	edgeDetect.setSearchMethod(index);
	edgeDetect.setRadiusSearch(EDGES_SEARCHRADIUS);
	edgeDetect.compute(boundaries);

	//prepare the detected points for display
	boundaryPoints.header=points->header;
	for(pcl::PointCloud<pcl::Boundary>::iterator it=boundaries.begin(); it<boundaries.end(); it++)
		if(it->boundary_point) boundaryPoints.push_back((*points)[(int)(it-boundaries.begin())]);

	//display the clouds we've created
	visualizer.publish(*points);
	visualizer2.publish(boundaryPoints);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle;

	//declare constants
	node->setParam("/xbot_surface/crop_xradius", 0.25); //the robot's radius
	node->setParam("/xbot_surface/crop_ymin", 0.3); //should be lt [floor_closey,floor_fary]
	node->setParam("/xbot_surface/crop_ymax", 1.0); //should be gt [floor_closey,floor_fary]
	node->setParam("/xbot_surface/crop_zmin", 0.0); //should be lt [floor_closez,floor_farz]
	node->setParam("/xbot_surface/crop_zmax", 1.5); //recommended gte [floor_closez,floor_farz], but shorten if the data is fading out to noise
	node->setParam("/xbot_surface/floor_closey", 0.3625); //y-coordinate of floor's close boundary, used to approximate its slope
	node->setParam("/xbot_surface/floor_closez", 0.8); //z-coordinate corresponding to above, used to approximate its slope
	node->setParam("/xbot_surface/floor_fary", 0.47); //y-coordinate of floor's far boundary, used to approximate its slope TODO recalculate
	node->setParam("/xbot_surface/floor_farz", 2.5); //z-coordinate of floor's far boundary, used to approximate its slope TODO recalculate
	node->setParam("/xbot_surface/floor_tolerance", 0.02); //maximum allowable y-coordinate devation of floor points
	node->setParam("/xbot_surface/edges_searchradius", 0.03);
	node->setParam("/xbot_surface/drive_linearspeed", 0.3); //TODO re-integrate
	node->setParam("/xbot_surface/drive_angularspeed", 0.4); //TODO re-integrate
	node->setParam("/xbot_surface/floor_transform", true); //whether to transform and flatten the floor, almost certainly improving the results
	node->setParam("/xbot_surface/drive_move", false); //set this to actually go somewhere! TODO re-integrate
	node->setParam("/xbot_surface/display_decisions", false); //limits point cloud output to still frames when the robot decides which way to go TODO re-integrate

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
	node->deleteParam("/xbot_surface/crop_xradius");
	node->deleteParam("/xbot_surface/crop_ymin");
	node->deleteParam("/xbot_surface/crop_ymax");
	node->deleteParam("/xbot_surface/crop_zmin");
	node->deleteParam("/xbot_surface/crop_zmax");
	node->deleteParam("/xbot_surface/floor_closey");
	node->deleteParam("/xbot_surface/floor_closez");
	node->deleteParam("/xbot_surface/floor_fary");
	node->deleteParam("/xbot_surface/floor_farz");
	node->deleteParam("/xbot_surface/floor_tolerance");
	node->deleteParam("/xbot_surface/edges_searchradius");
	node->deleteParam("/xbot_surface/drive_linearspeed");
	node->deleteParam("/xbot_surface/drive_angularspeed");
	node->deleteParam("/xbot_surface/floor_transform");
	node->deleteParam("/xbot_surface/drive_move");
	node->deleteParam("/xbot_surface/display_decisions");

	delete node;
}
