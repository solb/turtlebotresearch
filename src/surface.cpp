#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "geometry_msgs/Twist.h"

/**
Represents a request for a particular drive action, which may be to go straight, turn left, or turn right
*/
enum DriveAction
{
	FORWARD, LEFT, RIGHT
};

/**
Performs obstacle detection and avoidance using two algorithms simultaneously
*/
class TandemObstacleAvoidance
{
	private:
		ros::NodeHandle node;
		ros::Publisher velocity;
		ros::Publisher panorama; //downsampled cloud
		ros::Publisher height; //heightRange's region of interest
		DriveAction currentMOTION; //pilot's account of what was last done: detection algorithms should not modify!
		DriveAction directionsPrimary; //the height range algorithm's suggestion
		DriveAction directionsSecondary; //the ground edges algorithm's suggestion
		std::list<int> heightRangeFrontSamples;

		const char* directionRepresentation(DriveAction plan)
		{
			switch(plan)
			{
				case LEFT:
					return "LEFT";
				case RIGHT:
					return "RIGHT";
				default:
					return "FORWARD";
			}
		}

	public:
		/**
		Constructs the object, starts the algorithms, and blocks until the node is asked to shut down.  By default, all calculations are performed, but no commands are actually sent to the drive system unless the user sets the <tt>drive_move</tt> parameter to <tt>true</tt>, using the <tt>rosparam</tt> command, for instance.
		@param handle a <tt>NodeHandle</tt> defined with the nodespace containing the runtime parameters, including <tt>drive_move</tt>
		*/
		TandemObstacleAvoidance(ros::NodeHandle& handle):
			node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel", 1)), panorama(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("panorama", 1)), height(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("height", 1)), currentMOTION(FORWARD), directionsPrimary(FORWARD), directionsSecondary(FORWARD)
		{
			ros::MultiThreadedSpinner threads(3);
			ros::Subscriber heightRange=node.subscribe("/cloud_throttled", 1, &TandemObstacleAvoidance::heightRange, this);
			ros::Subscriber groundEdges=node.subscribe("/cloud_throttled", 1, &TandemObstacleAvoidance::groundEdges, this);
			ros::Timer pilot=node.createTimer(ros::Duration(0.1), &TandemObstacleAvoidance::pilot, this);

			threads.spin(); //blocks until the node is interrupted
		}

		/**
		Performs the primary obstacle detection and motion planning by downsampling the tunnel-like region in front of the robot and matching its approximate height and width
		@param cloud a Boost pointer to the <tt>PointCloud</tt> from the sensor
		*/
		void heightRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
		{
			//declare "constants," generated as described in pilot
			double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, HEIGHT_DOWNSAMPLING;
			int HEIGHT_SAMPLES;
			bool HEIGHT_VERBOSE;

			//populate "constants," generated as described in pilot
			node.getParamCached("crop_xradius", CROP_XRADIUS);
			node.getParamCached("crop_ymin", CROP_YMIN);
			node.getParamCached("crop_ymax", CROP_YMAX);
			node.getParamCached("crop_zmin", CROP_ZMIN);
			node.getParamCached("crop_zmax", CROP_ZMAX);
			node.getParamCached("height_downsampling", HEIGHT_DOWNSAMPLING);
			node.getParamCached("height_samples", HEIGHT_SAMPLES);
			node.getParamCached("height_verbose", HEIGHT_VERBOSE);

			//variable declarations/initializations
			pcl::PassThrough<pcl::PointXYZ> crop;
			pcl::VoxelGrid<pcl::PointXYZ> downsample;
			pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr front(new pcl::PointCloud<pcl::PointXYZ>);
			int averageObstacles=0; //number of points in our way after averaging our readings

			//downsample cloud
			downsample.setInputCloud(cloud);
			if(HEIGHT_DOWNSAMPLING>=0) downsample.setLeafSize((float)HEIGHT_DOWNSAMPLING, (float)HEIGHT_DOWNSAMPLING, (float)HEIGHT_DOWNSAMPLING);
			downsample.filter(*downsampled);

			//crop the cloud
			crop.setInputCloud(downsampled);
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

			if(currentMOTION!=0) heightRangeFrontSamples.clear(); //use straight snapshots while turning
			heightRangeFrontSamples.push_front(front->size());
			while(heightRangeFrontSamples.size()>(unsigned)HEIGHT_SAMPLES) heightRangeFrontSamples.pop_back(); //constrain our backlog

			//compute average number of points
			for(std::list<int>::iterator location=heightRangeFrontSamples.begin(); location!=heightRangeFrontSamples.end(); location++)
				averageObstacles+=*location;
			averageObstacles/=heightRangeFrontSamples.size();

			//let's DRIVE!
			if(averageObstacles>0) //something is in our way!
			{
				float centroidX=0;

				//compute the centroid of the detected points
				for(pcl::PointCloud<pcl::PointXYZ>::iterator point=front->begin(); point<front->end(); point++)
					centroidX+=point->x;
				centroidX/=front->size();

				if(HEIGHT_VERBOSE)
					ROS_INFO("HEIGHT RANGE :: Seeing %4d points in our way\n -> Centroid is at %.3f i", averageObstacles, centroidX);

				if(centroidX<0) //obstacle(s)'[s] centroid is off to left
					directionsPrimary=RIGHT;
				else
					directionsPrimary=LEFT;
			}
			else //nothing to see here
				directionsPrimary=FORWARD;

			//send our imagery to any connected visualizer
			panorama.publish(*downsampled);
			height.publish(*front);
		}

		/**
		Performs secondary obstacle detection and motion planning by detecting curvature changes on, boundaries of, and absense of the ground plane
		@param cloud a Boost pointer to the (organized) <tt>PointCloud</tt> from the sensor
		*/
		void groundEdges(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
		{
		}

		/**
		Integrates the decisions of the two obstacle detection methods and sends an appropriate drive command only if the <tt>drive_move</tt> parameter is set
		@param time the <tt>TimerEvent</tt> that triggered our schedule
		*/
		void pilot(const ros::TimerEvent& time)
		{
			//declare "constants," plus Vim macros to generate them from "populate 'constants'"
			#if 0
				Jldf,d/\af)C,
				$r;j
			#endif
			double DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
			bool DRIVE_MOVE, DRIVE_VERBOSE;

			//populate "constants," plus a Vim macro to generate them from "clean up parameters"
			#if 0
				>>>>^f.l6sgeteaCachedf"l"yyt"f)i, "ypvT,l~j
			#endif
			node.getParamCached("drive_linearspeed", DRIVE_LINEARSPEED);
			node.getParamCached("drive_angularspeed", DRIVE_ANGULARSPEED);
			node.getParamCached("drive_move", DRIVE_MOVE);
			node.getParamCached("drive_verbose", DRIVE_VERBOSE);

			//variable declarations
			DriveAction newMotion;
			geometry_msgs::Twist decision;

			//decide what to do, given the advice we've received
			if(DRIVE_VERBOSE)
				ROS_INFO("PILOT :: First recommendation says %s", directionRepresentation(directionsPrimary));
			newMotion=directionsPrimary;

			//don't reverse the direction of a turn
			if(newMotion!=FORWARD && currentMOTION!=FORWARD && newMotion!=currentMOTION)
			{
				if(DRIVE_VERBOSE) ROS_INFO("PILOT :: Overrode recommended oscillation");

				newMotion=currentMOTION; //keep rotating in the same direction we were
			}

			//make our move
			switch(newMotion)
			{
				case LEFT:
					if(DRIVE_VERBOSE) ROS_INFO("PILOT :: Turning %s", "LEFT");
					decision.angular.z=DRIVE_ANGULARSPEED;
					break;
				case RIGHT:
					if(DRIVE_VERBOSE) ROS_INFO("PILOT :: Turning %s", "RIGHT");
					decision.angular.z=-DRIVE_ANGULARSPEED;
					break;
				default:
					decision.linear.x=DRIVE_LINEARSPEED;
			}
			if(DRIVE_MOVE) velocity.publish(decision);

			//tell the obstacle detectors what we've done
			currentMOTION=newMotion;
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xbot_surface"); //string here is the node name
	ros::NodeHandle node("surface"); //string here is the namespace for parameters

	//default parameter values
	node.setParam("crop_xradius", 0.2); //should be slightly greater than robot's radius
	node.setParam("crop_ymin", -0.07); //should be slightly above robot's height
	node.setParam("crop_ymax", 0.35); //should be slightly above the ground's highest point
	node.setParam("crop_zmin", 0.0); //greater than zero excludes points close to robot
	node.setParam("crop_zmax", 1.5); //farthest to search for obstacles: lower for tighter maneuvering, higher for greater safety
	node.setParam("height_downsampling", 0.04); //less is more: should be low enough to eliminate noise from the region of interest (negative for [really bad] default)
	node.setParam("height_samples", 5); //number of samples to average: should be low enough to prevent false positives
	node.setParam("height_verbose", false);
	node.setParam("drive_linearspeed", 0.5);
	node.setParam("drive_angularspeed", 0.4);
	node.setParam("drive_move", false);
	node.setParam("drive_verbose", true);

	TandemObstacleAvoidance workhorse(node); //block to do obstacle avoidance

	//clean up parameters, plus a Vim macro to generate them from "default parameter values"
	#if 0
		^f.l3sdeletef,dt)f;C;j
	#endif
	node.deleteParam("crop_xradius");
	node.deleteParam("crop_ymin");
	node.deleteParam("crop_ymax");
	node.deleteParam("crop_zmin");
	node.deleteParam("crop_zmax");
	node.deleteParam("height_downsampling");
	node.deleteParam("height_samples");
	node.deleteParam("height_verbose");
	node.deleteParam("drive_linearspeed");
	node.deleteParam("drive_angularspeed");
	node.deleteParam("drive_move");
	node.deleteParam("drive_verbose");
}
