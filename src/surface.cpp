#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/extract_clusters.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle* node;
ros::Publisher outgoing; //target for visible point cloud
ros::Publisher drive; //target for steering instructions
std::list<int> frontSamples; //last few samples of what's immediately ahead
double steering=0; //current drive system angular command

/**
Calculates the average depth of the points in the given cloud.
Precondition: There is at least 1 point in the cloud!

@param obstructions The points about which we are concerned
@return The average z-coordinate of all provided points
*/
float averageDepth(pcl::PointCloud<pcl::PointXYZ>& obstructions)
{
	float depths=0;

	for(std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator spot=obstructions.begin(); spot<obstructions.end(); spot++)
		depths+=spot->z;
	depths/=obstructions.size(); //average

	return depths;
}

/**
Represents the distance between two groups of points.
*/
struct Distance
{
	public:
		/** The distance between the groups */
		float distance;
		/** The IDs of the associated objects */
		int oneObject, otherObject;
};

/**
Trims out all points in the list that at least as far away from the distant point as the close point.  The caller's copy of the closer point changes to become the closest member of the filtered subset of points.

@param world Point cloud containing at least the set of points about which we are concerned
@param object The indices of the points in the cloud on which we are operating; the rest will be ignored; this list will be trimmed to remove options that are known not to be the closest point
@param closer Minimally, the x- and z-coordinates of our current best-known point within the current object that is closest to the farther point; these will be revised with a better match
@param farther The point we're trying to get <tt>closer</tt> to be closer to
*/
void closerPoints(const pcl::PointCloud<pcl::PointXYZ>& world, std::list<int>& object, pcl::PointXYZ& closer, const pcl::PointXYZ& farther)
{
	pcl::PointXYZ oldCloser=closer;

	closer.x=oldCloser.x;
	closer.z=oldCloser.z;

	std::list<int>::iterator index=object.begin();
	while(index!=object.end())
	{
		const pcl::PointXYZ& point=world[*index];

		if(sqrt(pow(point.x-farther.x, 2)+pow(point.z-farther.z, 2))>=sqrt(pow(oldCloser.x-farther.x, 2)+pow(oldCloser.z-farther.z, 2))) //this point is at least as far away as the close one
			object.erase(index++); //exclude from all further consideration
		else //this point is oldCloser
		{
			if(sqrt(pow(point.x-farther.x, 2)+pow(point.z-farther.z, 2))<sqrt(pow(closer.x-farther.x, 2)+pow(closer.z-farther.z, 2))) //it's a better fit than our current favorite
			{
				closer.x=point.x;
				closer.z=point.z;
			}
			index++; //we'll keep this point for future consideration
		}
	}
}

/**
Calculates the distance between each pair of objects.

@param world A point cloud containing at least the relevant points
@param objects The <tt>PointIndices</tt> of the individual objects we're examining
@param centers The objects' center points
@param distances The calculated distances between the objects
@param oneObject (internal counter)
@param otherObject (internal counter)
*/
void findDistances(const pcl::PointCloud<pcl::PointXYZ>& world, const std::vector<pcl::PointIndices>& objects, const std::vector<pcl::PointXYZ>& centers, std::vector<Distance>& distances, const int oneObject=0, const int otherObject=0)
{
	if(oneObject!=otherObject) //I always expect the distance from any given object to itself to be 0...
	{
		std::list<int> one(objects[oneObject].indices.begin(), objects[oneObject].indices.end()), other(objects[otherObject].indices.begin(), objects[otherObject].indices.end()); //make temporary copies of our objects
		pcl::PointXYZ onePoint, otherPoint;
		Distance perpendicularDistance;

		onePoint.x=centers[oneObject].x;
		onePoint.z=centers[oneObject].z;
		otherPoint.x=centers[otherObject].x;
		otherPoint.z=centers[otherObject].z;

		while(one.size()>1 || other.size()>1)
		{
			if(one.size()>1) closerPoints(world, one, onePoint, otherPoint);
			if(other.size()>1) closerPoints(world, other, otherPoint, onePoint);
		}

		perpendicularDistance.distance=sqrt(pow(onePoint.x-otherPoint.x, 2)+pow(onePoint.z-otherPoint.z, 2));
		perpendicularDistance.oneObject=oneObject;
		perpendicularDistance.otherObject=otherObject;
		distances.push_back(perpendicularDistance);
	}

	if(otherObject<(int)objects.size()-1) //another otherObject left
		findDistances(world, objects, centers, distances, oneObject, otherObject+1); //look at the next otherObject
	else if(oneObject<(int)objects.size()-1) //another oneObject left
		findDistances(world, objects, centers, distances, oneObject+1, oneObject+1); //only consider the otherObject s that are greater than the new oneObject (to prevent computing duplicates)
}

/**
Called every time a new point cloud is available from the Kinect.  Responsible for our actual decisions and drive commands.

@param in The Kinect's cloud
*/
void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in)
{
	//these declarations may be partially generated from the read block below using the macro, but BEWARE OF TYPES:
	#if 0
	Jd2/\ui, f)d$
	#endif
	double YCROP_MIN, YCROP_MAX, ZCROP_MIN, ZCROP_MAX, DOWNSAMPLE_LEAFSIZE, CLUSTER_TOLERANCEFACTOR, DRIVE_RADIUS, DRIVE_OBSTACLE, DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	int CLUSTER_MINPOINTS, CLUSTER_HIDDENOBJECT, DRIVE_SAMPLES;
	bool DRIVE_MOVE, PRINT_DISTANCES, DISPLAY_TUNNELVISION, DISPLAY_DECISIONS;

	//read updated values for constants ... may be generated from declarations using the macro:
	#if 0
	^fsrg2f"F/l"vyt"f,wdt)"vPvb~f;d$a;j
	#endif
	node->getParam("/xbot_surface/ycrop_min", YCROP_MIN);
	node->getParam("/xbot_surface/ycrop_max", YCROP_MAX);
	node->getParam("/xbot_surface/zcrop_min", ZCROP_MIN);
	node->getParam("/xbot_surface/zcrop_max", ZCROP_MAX);
	node->getParam("/xbot_surface/downsample_leafsize", DOWNSAMPLE_LEAFSIZE);
	node->getParam("/xbot_surface/cluster_tolerancefactor", CLUSTER_TOLERANCEFACTOR);
	node->getParam("/xbot_surface/cluster_minpoints", CLUSTER_MINPOINTS);
	node->getParam("/xbot_surface/cluster_hiddenobject", CLUSTER_HIDDENOBJECT);
	node->getParam("/xbot_surface/drive_radius", DRIVE_RADIUS);
	node->getParam("/xbot_surface/drive_samples", DRIVE_SAMPLES);
	node->getParam("/xbot_surface/drive_obstacle", DRIVE_OBSTACLE);
	node->getParam("/xbot_surface/drive_linearspeed", DRIVE_LINEARSPEED);
	node->getParam("/xbot_surface/drive_angularspeed", DRIVE_ANGULARSPEED);
	node->getParam("/xbot_surface/drive_move", DRIVE_MOVE);
	node->getParam("/xbot_surface/print_distances", PRINT_DISTANCES);
	node->getParam("/xbot_surface/display_tunnelvision", DISPLAY_TUNNELVISION);
	node->getParam("/xbot_surface/display_decisions", DISPLAY_DECISIONS);

	//variable declarations/initializations
	pcl::PassThrough<pcl::PointXYZ> crop;
	pcl::VoxelGrid<pcl::PointXYZ> downsample;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
	pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr front(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr display;
	std::vector<pcl::PointIndices> clusters;
	std::vector<pcl::PointXYZ> clusterCenters; //each cluster's average point on the XZ-plane only
	std::vector<Distance> separations; //the separations on the XZ-plane between every pair of clusters
	geometry_msgs::Twist directions;
	int averageObstacles=0;

	//crop the cloud
	crop.setInputCloud(in);
	crop.setFilterFieldName("y");
	crop.setFilterLimits(YCROP_MIN, YCROP_MAX);
	crop.filter(*out);

	//downsample cloud
	downsample.setInputCloud(out);
	downsample.setLeafSize((float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE, (float)DOWNSAMPLE_LEAFSIZE);
	downsample.filter(*out);

	//separate distinct obstacles
	cluster.setInputCloud(out);
	cluster.setClusterTolerance(DOWNSAMPLE_LEAFSIZE*CLUSTER_TOLERANCEFACTOR);
	cluster.setMinClusterSize(CLUSTER_MINPOINTS);
	cluster.extract(clusters);
	ROS_INFO("Got %d clusters", (int)clusters.size());

	//calculate clusters' XZ-plane center points
	for(std::vector<pcl::PointIndices>::iterator group=clusters.begin(); group<clusters.end(); group++)
	{
		pcl::PointXYZ average;
		
		for(std::vector<int>::iterator point=group->indices.begin(); point<group->indices.end(); point++)
		{
			average.x+=(*out)[*point].x;
			average.z+=(*out)[*point].z;
		}
		average.x/=group->indices.size();
		average.z/=group->indices.size();

		clusterCenters.push_back(average);
	}

	//find the clusters' relative separations
	findDistances(*out, clusters, clusterCenters, separations);
	if(PRINT_DISTANCES)
		for(std::vector<Distance>::iterator span=separations.begin(); span<separations.end(); span++)
			ROS_INFO("Objects %d and %d are %f units apart%s", span->oneObject, span->otherObject, span->distance, span->distance>=2*DRIVE_RADIUS ? " and I COULD FIT BETWEEN!" : "");

	//mask out specific object if requested
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp=out;
	out.reset(new pcl::PointCloud<pcl::PointXYZ>);
	out->header=temp->header;
	for(std::vector<pcl::PointIndices>::iterator group=clusters.begin(); group<clusters.end(); group++)
		if(group-clusters.begin()!=CLUSTER_HIDDENOBJECT)
			for(std::vector<int>::iterator point=group->indices.begin(); point<group->indices.end(); point++)
				out->push_back((*temp)[*point]);
	display=DISPLAY_TUNNELVISION ? front : out;

	//create center "tunnel vision" region and store point count
	crop.setInputCloud(out);
	crop.setFilterFieldName("x");
	crop.setFilterLimits(-DRIVE_RADIUS, DRIVE_RADIUS);
	crop.filter(*front);

	//ignore distant obstructions so we don't turn too far in advance
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
		//spin off left and right vision fields for easy comparison
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

/**
Goes without saying.

@param argc Number of command-line arguments
@param argv The arguments themselves
*/
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
	node->setParam("/xbot_surface/cluster_tolerancefactor", 2.0);
	node->setParam("/xbot_surface/cluster_minpoints", 2);
	node->setParam("/xbot_surface/cluster_hiddenobject", -1); //-1 is a safe sentinel for none
	node->setParam("/xbot_surface/drive_radius", 0.25); //lateral radius from center of boundaries between navigational thirds
	node->setParam("/xbot_surface/drive_samples", 5); //number of sensor readings to average in order to filter out noise (for front region only)
	node->setParam("/xbot_surface/drive_obstacle", 1); //minimum number of points that are considered an obstacle to our forward motion
	node->setParam("/xbot_surface/drive_linearspeed", 0.3);
	node->setParam("/xbot_surface/drive_angularspeed", 0.4);
	node->setParam("/xbot_surface/drive_move", false); //whether or not to actually move
	node->setParam("/xbot_surface/print_distances", false); //whether to spam the console with *slow* readouts of the distances between detected clusters
	node->setParam("/xbot_surface/display_tunnelvision", false); //sends the straight-ahead, shortened view instead of long, panaramic one
	node->setParam("/xbot_surface/display_decisions", false); //limits point cloud output to still frames when the robot decides which way to go

	//request and pass messages
	ros::Subscriber incoming=node->subscribe("/cloud_throttled", 1, callback);
	outgoing=node->advertise< pcl::PointCloud<pcl::PointXYZ> >("/cloud_surfaces", 1);
	drive=node->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::spin(); //put us in a callback loop until ROS deems us unworthy

	//clean up constants ... may be generated from declarations using the macro:
	#if 0
	:s/set/deletef,dt)f;d$a;j
	#endif
	node->deleteParam("/xbot_surface/ycrop_min");
	node->deleteParam("/xbot_surface/ycrop_max");
	node->deleteParam("/xbot_surface/zcrop_min");
	node->deleteParam("/xbot_surface/zcrop_max");
	node->deleteParam("/xbot_surface/downsample_leafsize");
	node->deleteParam("/xbot_surface/cluster_tolerancefactor");
	node->deleteParam("/xbot_surface/cluster_minpoints");
	node->deleteParam("/xbot_surface/cluster_hiddenobject");
	node->deleteParam("/xbot_surface/drive_radius");
	node->deleteParam("/xbot_surface/drive_samples");
	node->deleteParam("/xbot_surface/drive_samples");
	node->deleteParam("/xbot_surface/drive_obstacle");
	node->deleteParam("/xbot_surface/drive_linearspeed");
	node->deleteParam("/xbot_surface/drive_angularspeed");
	node->deleteParam("/xbot_surface/drive_move");
	node->deleteParam("/xbot_surface/print_distances");
	node->deleteParam("/xbot_surface/display_tunnelvision");
	node->deleteParam("/xbot_surface/display_decisions");

	delete node; //let's not clutter up the heap...
}
