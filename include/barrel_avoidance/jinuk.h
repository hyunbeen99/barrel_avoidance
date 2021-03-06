#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Point.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "velodyne_pointcloud/point_types.h"

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

#define _USE_MATH_DEFINES
#define DIST 3.5

typedef pcl::PointXYZI PointType;

class StaticAvoidance{
private:
	// ros
    ros::NodeHandle nh_;

	// publisher
    ros::Publisher pub_;
    ros::Publisher marker_pub_;

	// subscriber
    ros::Subscriber sub_;
	ros::Subscriber imu_sub_;

	// message
    ackermann_msgs::AckermannDriveStamped ackerData_;
	
	// data
    vector<geometry_msgs::Point> obstacles_;

	// value
    int status_;

	int speed_;
	int steer_;

	// imu
	double roll, pitch, yaw;
    double yaw_deg_;
    double init_yaw_;
	bool init_flag_;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &input);
    void imuCallback(const sensor_msgs::ImuConstPtr &imu);
	
	void visualize(vector<geometry_msgs::Point> input_points);
    void visualize(geometry_msgs::Point point);

    void run();
    
    double getDist(geometry_msgs::Point p);
    double calcSteer(geometry_msgs::Point p);
	void print(vector<geometry_msgs::Point> points);

    StaticAvoidance() {
        initSetup();
    }
};

