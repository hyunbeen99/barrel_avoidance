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
//#include "kuuve_control/Kuuve.h"
#include "std_msgs/Bool.h"

#define _USE_MATH_DEFINES
#define STANDARD_DIST 4.5
#define FIX_DIST 4.0

#define LEFT_FIRST 100
#define RIGHT_FIRST 200

#define INIT_SPEED 1.5
#define INIT_STEER 0.0

#define SPEED1 1.5 
#define SPEED3 1.5

#define MINSTEER -25
#define MAXSTEER 25

#define LEFT_ANG_GAP 22
#define RIGHT_ANG_GAP 30

typedef pcl::PointXYZI PointType;

class StaticAvoidance{

private:
    ros::NodeHandle nh_;

	//publisher
    ros::Publisher pub_;
    ros::Publisher marker_pub_;
    ros::Publisher state_pub_;

	//subscriber
    ros::Subscriber sub_;
	ros::Subscriber imu_sub_;
	ros::Subscriber state_sub_;
	
	//message
    ackermann_msgs::AckermannDriveStamped ackerData_;
	std_msgs::Bool isStaticFinished_;
	
	//data
    vector<geometry_msgs::Point> center_point_;
    vector<geometry_msgs::Point> fixed_point_;

    //flag
	bool get_first_imu;
	bool get_second_imu;
	int obs_align_;
	int fix_obs_check_;

	//values
	int cur_state_;
    int status_;
	int steer;

	double roll, pitch, yaw;
    double yaw_degree_;
	double dist;

	double init_yaw_;
	double second_yaw_;


public:
    void initSetup();

  //  void stateCallback(const kuuve_control::Kuuve::ConstPtr &state);
    void imuCallback(const sensor_msgs::ImuConstPtr &imu);
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &input);

	
	void visualize(vector<geometry_msgs::Point> input_points);
    void visualize(geometry_msgs::Point point);

    void run();
	void fixObstacles();
    
    double getDist(geometry_msgs::Point p);
    double calcSteer(geometry_msgs::Point p);

    StaticAvoidance() {
        initSetup();
    }

};

