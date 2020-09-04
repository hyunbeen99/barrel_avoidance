#include "ros/ros.h"
//PCL_ROS
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

typedef pcl::PointXYZI PointType;

class Euclideon{

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
};

void Euclideon::initSetup(){
    sub_ = nh_.subscribe("velodyne_points",100,&Euclideon::pointCallback,this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result", 100);
}


void Euclideon::pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    //Convert sensor_msgs to pcl/pointcloud
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>), cloud_f (new pcl::PointCloud<PointType>), cloud_f2 (new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Data container used
    // Create the segmentation object for the planar model and set all the parameters
    /*pcl::SACSegmentation<PointType> seg;
    // Remove plane

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create a pcl object to hold the ransac filtered object
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>()); 

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);    
    seg.setMaxIterations(10);
    seg.setDistanceThreshold(0.04);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);*/
    //downsampling
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f,0.05f,0.05f);
    vg.filter(*cloud_f2);
    cout << "filterd point size: " << cloud_f2 -> points.size() << "points" << endl;
    
    
    //Creating KDtree object for the search method of the extraction
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree -> setInputCloud(cloud_f2);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.3); //20cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(2500);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_f2);
    ec.extract(cluster_indices);

    cout << "Number of clusters is equal to " << cluster_indices.size() << endl;

    pcl::PointCloud<PointType> Result_cloud;
    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            PointType pt2;
            pt2.x = cloud_f2->points[*pit].x, pt2.y = cloud_f2->points[*pit].y, pt2.z = cloud_f2->points[*pit].z;
            pt2.intensity = (float)(j+1);

            Result_cloud.push_back(pt2);
        }
        j++; 
    }

    //Convert To ROS data type
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(Result_cloud, cloud_p);

    sensor_msgs::PointCloud2 result;
    pcl_conversions::fromPCL(cloud_p, result);
    result.header.frame_id = "velodyne";
    pub_.publish(result);

    ROS_INFO("pulished it");
}

int main (int argc, char** argv)
{
    ros::init(argc,argv,"euclideon_node");
    Euclideon EC;
    EC.initSetup();
    ros::spin();
}
