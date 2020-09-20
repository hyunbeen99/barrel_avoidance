#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
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

#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "math.h"
#include "vector"

#define _USE_MATH_DEFINES
using namespace std;
typedef pcl::PointXYZI PointType;

class Cluster {
	public:
		vector<geometry_msgs::Point> cluster(const sensor_msgs::PointCloud2ConstPtr &input, double x_min, double x_max, double y_min, double y_max);
		vector<geometry_msgs::Point> filter(vector<geometry_msgs::Point> points);
		vector<geometry_msgs::Point> sort(vector<geometry_msgs::Point> list);
		vector<geometry_msgs::Point> sortFilter(vector<geometry_msgs::Point> list);
		double getDist(geometry_msgs::Point point);
};
