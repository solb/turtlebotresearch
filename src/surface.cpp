#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/features/organized_edge_detection.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle* node;
ros::Publisher visualizer;
ros::Publisher visualizer2;
ros::Publisher drive;
double last_FLOOR_CLOSEY=0, last_FLOOR_CLOSEZ=0, last_FLOOR_FARY=0, last_FLOOR_FARZ=0; //only recalculate the below when necessary
double FLOOR_SLOPE, FLOOR_YINTERCEPT; //model the floor's location
int lastPreferredDirection=1; //which way we'd like to turn (-1 for left, 1 for right)
geometry_msgs::Twist directions;

void pilot(const ros::TimerEvent& happening)
{
	bool DRIVE_MOVE;

	node->getParamCached("/xbot_surface/drive_move", DRIVE_MOVE);
	if(DRIVE_MOVE) drive.publish(directions);
}

void process(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& in)
{
	//these "constant" declarations may be partially generated from the read block below using the following two macros, but be careful of types:
	#if 0
	Jd3/\ui, f)d$
	^ExA;j
	#endif
	double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, FLOOR_CLOSEY, FLOOR_CLOSEZ, FLOOR_FARY, FLOOR_FARZ, FLOOR_HIGHTOLERANCE, FLOOR_LOWTOLERANCE, EDGES_SEARCHRADIUS, EDGES_NORMALSMOOTHING, EDGES_THRESHOLDLOWER, EDGES_THRESHOLDHIGHER, BOUNDARY_BUMPERFRONTAL, BOUNDARY_BUMPERLATERAL, OUTLIERS_SEARCHRADIUS, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED, DRIVE_REVERSESPEED;
	int EDGES_DETECTIONTYPE, EDGES_NORMALESTIMATION, OUTLIERS_MINNEIGHBORS, DANGER_FLOORSIZE;
	bool FLOOR_TRANSFORM, OUTLIERS_REMOVE, PRINT_DECISIONS, DISPLAY_DECISIONS;

	//read updated values for "constants" ... may be generated from declarations using the macro:
	#if 0
	^fsrgeaCached2f"F/l"vyt"f,wdt)"vPvb~f;d$a;j
	#endif
	node->getParamCached("/xbot_surface/crop_xradius", CROP_XRADIUS);
	node->getParamCached("/xbot_surface/crop_ymin", CROP_YMIN);
	node->getParamCached("/xbot_surface/crop_ymax", CROP_YMAX);
	node->getParamCached("/xbot_surface/crop_zmin", CROP_ZMIN);
	node->getParamCached("/xbot_surface/crop_zmax", CROP_ZMAX);
	node->getParamCached("/xbot_surface/floor_closey", FLOOR_CLOSEY);
	node->getParamCached("/xbot_surface/floor_closez", FLOOR_CLOSEZ);
	node->getParamCached("/xbot_surface/floor_fary", FLOOR_FARY);
	node->getParamCached("/xbot_surface/floor_farz", FLOOR_FARZ);
	node->getParamCached("/xbot_surface/floor_hightolerance", FLOOR_HIGHTOLERANCE);
	node->getParamCached("/xbot_surface/floor_lowtolerance", FLOOR_LOWTOLERANCE);
	node->getParamCached("/xbot_surface/edges_searchradius", EDGES_SEARCHRADIUS);
	node->getParamCached("/xbot_surface/edges_normalsmoothing", EDGES_NORMALSMOOTHING);
	node->getParamCached("/xbot_surface/edges_thresholdlower", EDGES_THRESHOLDLOWER);
	node->getParamCached("/xbot_surface/edges_thresholdhigher", EDGES_THRESHOLDHIGHER);
	node->getParamCached("/xbot_surface/boundary_bumperfrontal", BOUNDARY_BUMPERFRONTAL);
	node->getParamCached("/xbot_surface/boundary_bumperlateral", BOUNDARY_BUMPERLATERAL);
	node->getParamCached("/xbot_surface/outliers_searchradius", OUTLIERS_SEARCHRADIUS);
	node->getParamCached("/xbot_surface/drive_linearspeed", DRIVE_LINEARSPEED);
	node->getParamCached("/xbot_surface/drive_angularspeed", DRIVE_ANGULARSPEED);
	node->getParamCached("/xbot_surface/drive_reversespeed", DRIVE_REVERSESPEED);
	node->getParamCached("/xbot_surface/edges_detectiontype", EDGES_DETECTIONTYPE);
	node->getParamCached("/xbot_surface/edges_normalestimation", EDGES_NORMALESTIMATION);
	node->getParamCached("/xbot_surface/boundary_bumperfrontal", BOUNDARY_BUMPERFRONTAL);
	node->getParamCached("/xbot_surface/boundary_bumperlateral", BOUNDARY_BUMPERLATERAL);
	node->getParamCached("/xbot_surface/outliers_minneighbors", OUTLIERS_MINNEIGHBORS);
	node->getParamCached("/xbot_surface/danger_floorsize", DANGER_FLOORSIZE);
	node->getParamCached("/xbot_surface/floor_transform", FLOOR_TRANSFORM);
	node->getParamCached("/xbot_surface/outliers_remove", OUTLIERS_REMOVE);
	node->getParamCached("/xbot_surface/print_decisions", PRINT_DECISIONS);
	node->getParamCached("/xbot_surface/display_decisions", DISPLAY_DECISIONS);

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
	pcl::PassThrough<pcl::PointXYZRGB> crop;
	pcl::OrganizedEdgeDetection<pcl::PointXYZRGB, pcl::Label> detect;
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> remove;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Label> edgePoints;
	std::vector<pcl::PointIndices> edges;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr navigation(new pcl::PointCloud<pcl::PointXYZRGB>);
	int trueFloorPoints=0; //size of the floor itself, not including any obstacles
	double trueFloorXTotal=0; //total of all the floor's x-coordinates

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
	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator location=points->begin(); location<points->end(); location++)
	{
		double distanceFromFloorPlane=fabs(location->y/*point's actual y-coordinate*/ - (FLOOR_SLOPE*location->z+FLOOR_YINTERCEPT)/*floor's expected y-coordinate*/);
		if(distanceFromFloorPlane>FLOOR_HIGHTOLERANCE) //this point isn't anywhere near the floor
		{ //these aren't the points we're looking for
			location->x=std::numeric_limits<float>::quiet_NaN();
			location->y=std::numeric_limits<float>::quiet_NaN();
			location->z=std::numeric_limits<float>::quiet_NaN();
		}
		else //it is quite close to the floor
		{
			if(distanceFromFloorPlane<=FLOOR_LOWTOLERANCE && fabs(location->x)<CROP_XRADIUS-BOUNDARY_BUMPERLATERAL && location->z>CROP_ZMIN+BOUNDARY_BUMPERFRONTAL && location->z<CROP_ZMAX-BOUNDARY_BUMPERFRONTAL) //actually part of the floor and in the subregion where we do not tolerate intruding plane edges
			{
				trueFloorPoints++;
				trueFloorXTotal+=location->x;
			}

			if(FLOOR_TRANSFORM)
			{
				location->z+=FLOOR_SLOPE/location->y; //transform the floor onto the XZ plane
				location->y=0; //flatten it
			}
		}
	}
	if(FLOOR_TRANSFORM)
	{
		CROP_ZMIN+=FLOOR_SLOPE/(FLOOR_SLOPE*CROP_ZMIN+FLOOR_YINTERCEPT);
		CROP_ZMAX+=FLOOR_SLOPE/(FLOOR_SLOPE*CROP_ZMAX+FLOOR_YINTERCEPT);
	}

	//optimize constants TODO remove
	//double TEMP=10.0;
	//bool TEMP=false;
	//if(node->hasParam("/xbot_test")) node->getParam("/xbot_test", TEMP);

	if(PRINT_DECISIONS) ROS_INFO("Seeing %5d floor points", trueFloorPoints);
	if(trueFloorPoints>=DANGER_FLOORSIZE) //don't waste time if we're blind
	{
		//detect edges
		detect.setInputCloud(points);
		detect.setEdgeType(EDGES_DETECTIONTYPE);
		if(EDGES_SEARCHRADIUS>=0) detect.setDepthDisconThreshold((float)EDGES_SEARCHRADIUS);
		if(EDGES_NORMALESTIMATION>=0) detect.setHighCurvatureNormalEstimationMethod((pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::NormalEstimationMethod)EDGES_NORMALESTIMATION);
		if(EDGES_NORMALSMOOTHING>=0) detect.setHighCurvatureNormalSmoothingSize((float)EDGES_NORMALSMOOTHING);
		if(EDGES_THRESHOLDLOWER>=0) detect.setHighCurvatureEdgeThresholdLower((float)EDGES_THRESHOLDLOWER);
		if(EDGES_THRESHOLDHIGHER>=0) detect.setHighCurvatureEdgeThresholdHigher((float)EDGES_THRESHOLDHIGHER);
		detect.compute(edgePoints, edges);

		if(PRINT_DECISIONS)
		{
			std::stringstream edgy;
			edgy<<" ... broken down as";
			for(std::vector<pcl::PointIndices>::iterator edgeType=edges.begin(); edgeType<edges.end(); edgeType++)
				edgy<<", "<<setw(4)<<edgeType->indices.size();
			ROS_INFO("%s", edgy.str().c_str());
		}

		//assemble the detected points
		navigation->header=points->header;
		for(std::vector<int>::iterator pointIndex=edges[0].indices.begin(); pointIndex<edges[0].indices.end(); pointIndex++) //copy NaN/plane boundaries
			if(fabs((*points)[*pointIndex].x)<CROP_XRADIUS-BOUNDARY_BUMPERLATERAL && (*points)[*pointIndex].z>CROP_ZMIN+BOUNDARY_BUMPERFRONTAL && (*points)[*pointIndex].z<CROP_ZMAX-BOUNDARY_BUMPERFRONTAL) //point is far enough from the edge
				navigation->push_back((*points)[*pointIndex]);
		for(std::vector<pcl::PointIndices>::iterator edge=edges.begin()+1; edge<edges.end(); edge++) //copy every other edge type
			for(std::vector<int>::iterator pointIndex=edge->indices.begin(); pointIndex<edge->indices.end(); pointIndex++)
					navigation->push_back((*points)[*pointIndex]);

		//eliminate outliers
		if(OUTLIERS_REMOVE && navigation->size()>0)
		{
			remove.setInputCloud(navigation);
			remove.setRadiusSearch((float)OUTLIERS_SEARCHRADIUS);
			if(OUTLIERS_MINNEIGHBORS>=0) remove.setMinNeighborsInRadius(OUTLIERS_MINNEIGHBORS);
			remove.filter(*navigation);
		}
	}

	//plan our next move
	if(navigation->size()>0) //something in our way
	{
		float centroidX=0;

		//where are our obstructions centered?
		for(pcl::PointCloud<pcl::PointXYZRGB>::iterator point=navigation->begin(); point<navigation->end(); point++)
			centroidX+=point->x;
		centroidX/=navigation->size();
		if(PRINT_DECISIONS) ROS_INFO("Seeing %3lu offending points centered at %.3f i", navigation->size(), centroidX);

		if(directions.angular.z==0) //don't pick a new direction unless we aren't turning already
		{
			directions.linear.x=0;
			if(centroidX<0) //offenders mostly to our left
			{
				directions.angular.z=-DRIVE_ANGULARSPEED; //move right
				lastPreferredDirection=1; //continue even if we lose sight of the floor in the next frame
				if(PRINT_DECISIONS) ROS_INFO(" ... Moving RIGHT");
			}
			else /*centroidX>=0*/
			{
				directions.angular.z=DRIVE_ANGULARSPEED; //move left
				lastPreferredDirection=-1; //continue even if we lose sight of the floor in the next frame
				if(PRINT_DECISIONS) ROS_INFO(" ... Moving LEFT");
			}
		}

		//in case we lose sight of the floor in the next frame, we'll keep turning in this direction
		
	}
	else if(trueFloorPoints<DANGER_FLOORSIZE) //where'd the floor go?
	{
		if(PRINT_DECISIONS) ROS_INFO("Not seeing much ground; executing emergency evasive maneuvers!");
		directions.linear.x=0; //we're too close
		directions.angular.z=-lastPreferredDirection*DRIVE_ANGULARSPEED;
	}
	else //we're all clear
	{
		directions.linear.x=DRIVE_LINEARSPEED; //go
		directions.angular.z=0; //straight on

		//in case we lose sight of the floor in the next frame, we'll turn toward the direction where more of it is visible
		lastPreferredDirection=trueFloorXTotal/trueFloorPoints>0 ? 1 : -1;
	}

	//display the clouds we've created
	if(!DISPLAY_DECISIONS || navigation->size()==0)
	{
		visualizer.publish(*points);
		visualizer2.publish(*navigation);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface");
	node=new ros::NodeHandle;

	node->deleteParam("/xbot_test"); //for quickly optimizing constants TODO remove

	//declare constants
	node->setParam("/xbot_surface/crop_xradius", 0.3); //the robot's radius
	node->setParam("/xbot_surface/crop_ymin", 0.3); //should be lt [floor_closey,floor_fary]
	node->setParam("/xbot_surface/crop_ymax", 1.0); //should be gt [floor_closey,floor_fary]
	node->setParam("/xbot_surface/crop_zmin", 0.8); //should be lte [floor_closez,floor_farz]
	node->setParam("/xbot_surface/crop_zmax", 1.5); //recommended gte [floor_closez,floor_farz], but shorten if the data is fading out to noise
	node->setParam("/xbot_surface/floor_closey", 0.3625); //y-coordinate of floor's close boundary, used to approximate its slope
	node->setParam("/xbot_surface/floor_closez", 0.8); //z-coordinate corresponding to above, used to approximate its slope
	node->setParam("/xbot_surface/floor_fary", 0.47); //y-coordinate of floor's far boundary, used to approximate its slope TODO recalculate
	node->setParam("/xbot_surface/floor_farz", 2.5); //z-coordinate of floor's far boundary, used to approximate its slope TODO recalculate
	node->setParam("/xbot_surface/floor_hightolerance", 0.1); //maximum allowable y-coordinate devation of considered points
	node->setParam("/xbot_surface/floor_lowtolerance", 0.03); //maximum allowable y-coordinate devation of floor points
	node->setParam("/xbot_surface/edges_searchradius", -0.045); //higher number means smaller false positive patches, but slower processing speed; negative to use the default
	node->setParam("/xbot_surface/edges_normalsmoothing", -1.0); //negative to use the default
	node->setParam("/xbot_surface/edges_thresholdlower", 1.0); //negative to use the default
	node->setParam("/xbot_surface/edges_thresholdhigher", 1.7); //negative to use the default
	node->setParam("/xbot_surface/boundary_bumperfrontal", 0.1); //the tolerance from the front and back edges of the cropped floor area that is considered a normal plane boundary
	node->setParam("/xbot_surface/boundary_bumperlateral", 0.02); //the tolerance from the left and right back edges of the cropped area that is considered a normal plane boundary, which is best gt boundary_bumperfrontal
	node->setParam("/xbot_surface/outliers_searchradius", 0.05); //set high enough that separate clusters won't run into each other
	node->setParam("/xbot_surface/drive_linearspeed", 0.5);
	node->setParam("/xbot_surface/drive_angularspeed", 0.25);
	node->setParam("/xbot_surface/edges_detectiontype", pcl::OrganizedEdgeDetection<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_NAN_BOUNDARY+pcl::OrganizedEdgeDetection<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_HIGH_CURVATURE); //as defined in OrganizedEdgeDetection
	node->setParam("/xbot_surface/edges_normalestimation", -1); //as defined in IntegralImageNormalEstimation; negative to use the default
	node->setParam("/xbot_surface/outliers_minneighbors", 6); //will trim out obstacles if set unduly high; negative to use the default
	node->setParam("/xbot_surface/danger_floorsize", 1); //minimum number of visible floor points before we decide that something (obstacle? hole?) is right in front of us
	node->setParam("/xbot_surface/floor_transform", false); //whether to transform and flatten the floor, almost certainly improving the results
	node->setParam("/xbot_surface/outliers_remove", true); //whether to filter out suspected false positives
	node->setParam("/xbot_surface/print_decisions", true); //report on our rationale whenever we decide to turn
	node->setParam("/xbot_surface/display_decisions", false); //limits point cloud output to still frames when the robot decides which way to go
	//go in the pilot callback:
	node->setParam("/xbot_surface/drive_move", false); //set this to actually go somewhere!

	//request and pass messages
	ros::MultiThreadedSpinner caller(2);
	ros::Subscriber incoming=node->subscribe("/cloud_throttled", 1, process);
	ros::Timer commander=node->createTimer(ros::Duration(0.1), pilot);
	visualizer=node->advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/cloud_surfaces", 1);
	visualizer2=node->advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/cloud_hull", 1);
	drive=node->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	caller.spin();

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
	node->deleteParam("/xbot_surface/floor_hightolerance");
	node->deleteParam("/xbot_surface/floor_lowtolerance");
	node->deleteParam("/xbot_surface/edges_searchradius");
	node->deleteParam("/xbot_surface/edges_normalsmoothing");
	node->deleteParam("/xbot_surface/edges_thresholdlower");
	node->deleteParam("/xbot_surface/edges_thresholdhigher");
	node->deleteParam("/xbot_surface/boundary_bumperfrontal");
	node->deleteParam("/xbot_surface/boundary_bumperlateral");
	node->deleteParam("/xbot_surface/outliers_searchradius");
	node->deleteParam("/xbot_surface/drive_linearspeed");
	node->deleteParam("/xbot_surface/drive_angularspeed");
	node->deleteParam("/xbot_surface/drive_reversespeed");
	node->deleteParam("/xbot_surface/edges_detectiontype");
	node->deleteParam("/xbot_surface/edges_normalestimation");
	node->deleteParam("/xbot_surface/outliers_minneighbors");
	node->deleteParam("/xbot_surface/danger_floorsize");
	node->deleteParam("/xbot_surface/floor_transform");
	node->deleteParam("/xbot_surface/outliers_remove");
	node->deleteParam("/xbot_surface/print_decisions");
	node->deleteParam("/xbot_surface/display_decisions");
	//go in the pilot callback:
	node->deleteParam("/xbot_surface/drive_move");

	delete node;
}
