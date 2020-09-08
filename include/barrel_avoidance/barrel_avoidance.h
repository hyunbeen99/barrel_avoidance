#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Point.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"

#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include "sensor_msgs/Imu.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose2D.h"

#include "cmath"
#include "math.h"
#include "vector"
#include "tuple"

#include "clustering.cpp"

#define ESCAPE_LEFT_STEER -15
#define ESCAPE_RIGHT_STEER 15

#define DEFAULT_SPEED 1.0
#define AVOID_SPEED 1.0
#define ESCAPE_SPEED 1.0

#define DIST 2.0

#define LEFT 100
#define RIGHT 200

#define _USE_MATH_DEFINES
typedef pcl::PointXYZI PointType;

class StaticAvoidance{
private:
	enum {
		AVOID_CLOSE = 0,
		AVOID_FAR = 1,
		ESCAPE = 2
	};

	// node handle
    ros::NodeHandle nh_;

	// publishers
    ros::Publisher pub_;
    ros::Publisher marker_pub_;

	// subsribers
    ros::Subscriber sub_;

	// data
    vector<geometry_msgs::Point> obstacles_;
    ackermann_msgs::AckermannDriveStamped ackerData_;
        
	// values
    int status_;

	double steer_;
	double speed_;

	int obs_count_;
	bool init_flag_;
	bool obs_status_flag_;

	int obs_status_;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &input);

    void run();
	void go();

	void avoidClose();
	void avoidFar();
	void escape();
	void fixObstacleStatus();
    
    double calcSteer(geometry_msgs::Point p);
	double calcAngle(geometry_msgs::Point p);
    double getDist(geometry_msgs::Point p);
	
	void visualize(vector<geometry_msgs::Point> obs_points);
    void visualize(geometry_msgs::Point point);
	void printStatus();

    StaticAvoidance() {
        initSetup();
    }
};

