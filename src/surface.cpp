#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "geometry_msgs/Twist.h"

/**
Performs obstacle detection and avoidance using two algorithms simultaneously
*/
class TandemObstacleAvoidance
{
	private:
		ros::NodeHandle node;
		ros::Publisher velocity;
		geometry_msgs::Twist drive; //motion command

	public:
		/**
		Constructs the object, starts the algorithms, and blocks until the node is asked to shut down.  By default, all calculations are performed, but no commands are actually sent to the drive system unless the user sets the <tt>move</tt> parameter to <tt>true</tt>, using the <tt>rosparam</tt> command, for instance.
		@param handle a <tt>NodeHandle</tt> defined with the nodespace containing the runtime parameters, including <tt>move</tt>
		*/
		TandemObstacleAvoidance(ros::NodeHandle& handle):
			node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel", 1))
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
			drive.angular.z=0.05; //left
		}

		/**
		Performs secondary obstacle detection and motion planning by detecting curvature changes on, boundaries of, and absense of the ground plane
		@param cloud a Boost pointer to the (organized) <tt>PointCloud</tt> from the sensor
		*/
		void groundEdges(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
		{
			drive.angular.z=-0.05; //right
		}

		/**
		Integrates the decisions of the two obstacle detection methods and sends an appropriate drive command only if the <tt>move</tt> parameter is set
		@param time the <tt>TimerEvent</tt> that triggered our schedule
		*/
		void pilot(const ros::TimerEvent& time)
		{
			//declare "constants," plus Vim macros to generate them from "populate 'constants'"
			#if 0
				Jldf,d/\af)C,
				$r;j
			#endif
			bool MOVE;

			//populate "constants," plus a Vim macro to generate them from "clean up parameters"
			#if 0
				>>>>^f.l6sgeteaCachedf"l"yyt"f)i, "ypvT,l~j
			#endif
			node.getParamCached("move", MOVE);

			if(MOVE) velocity.publish(drive);
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xbot_surface"); //string here is the node name
	ros::NodeHandle node("surface"); //string here is the namespace for parameters

	//default parameter values
	node.setParam("move", false);

	TandemObstacleAvoidance workhorse(node); //block to do obstacle avoidance

	//clean up parameters, plus a Vim macro to generate them from "default parameter values"
	#if 0
		^f.l3sdeletef,dt)f;C;j
	#endif
	node.deleteParam("move");
}
