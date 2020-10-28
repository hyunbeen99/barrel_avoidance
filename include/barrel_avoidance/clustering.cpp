#include "barrel_avoidance/clustering.h"

vector<geometry_msgs::Point> Cluster::cluster(const sensor_msgs::PointCloud2ConstPtr &input, double x_min, double x_max, double y_min, double y_max){
    pcl::PointCloud<PointType>::Ptr msg (new pcl::PointCloud<PointType>), cloud (new pcl::PointCloud<PointType>), cloud_filterd (new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*input, *msg);
    for (size_t i=0;i<msg->points.size();i++){
		PointType point;
		point.x = msg->points[i].x*cos(12*M_PI/180) + msg->points[i].z*sin(12*M_PI/180); 
		point.y = msg->points[i].y;
		point.z = -msg->points[i].x*sin(12*M_PI/180) + msg->points[i].z*cos(12*M_PI/180); 
		point.intensity = msg->points[i].intensity;
		cloud->points.push_back(point);

	}
	cloud->header.frame_id = "velodyne";

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud);
    
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.7, 10);
    pass.filter(*cloud);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create a pcl object to hold the ransac filtered object
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>()); 
    
    pcl::SACSegmentation<PointType> seg;
    
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);
    seg.setMaxIterations(100);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_filterd);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree -> setInputCloud(cloud_filterd);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(1); //30cm
    ec.setMinClusterSize(6);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filterd);
    ec.extract(cluster_indices);


    cloud_filterd->header.frame_id = "velodyne";
    pcl::PointCloud<PointType> cluster_cloud1, cluster_cloud2, cluster_cloud3, cluster_cloud4, Result_cloud;
   // cout << "Number of clusters is equal to " << cluster_indices.size() << endl;
    int j = 0;

    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        if(it == cluster_indices.begin()){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);

            	cluster_cloud1.push_back(pt2);}
		}
        
		if(it == cluster_indices.begin()+1){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud2.push_back(pt2);}
		}
		
        if(it == cluster_indices.begin()+2){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud3.push_back(pt2);}
		}
        if(it == cluster_indices.begin()+3){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud4.push_back(pt2);}
		}
        j++; 
    }

    double sum_x=0; 
    double sum_y=0;
    double sum_z=0;
	visualization_msgs::Marker mean_point;
	mean_point.header.frame_id = "velodyne";
	mean_point.header.stamp = ros::Time::now();
	mean_point.ns = "points";
	mean_point.action = visualization_msgs::Marker::ADD;
	mean_point.type = visualization_msgs::Marker::POINTS;
	mean_point.pose.orientation.w = 1;
	mean_point.id = 0;
	mean_point.color.g = 1.0f; 
	mean_point.color.a = 1.0;
    mean_point.scale.x = 0.1;
    mean_point.scale.y = 0.1;
	
	vector<geometry_msgs::Point> mean_p;
    if (cluster_cloud1.size() != 0){
        for (size_t i=0; i<cluster_cloud1.size(); i++){
           // cout << cluster_cloud2[i].x << endl;
            sum_x += cluster_cloud1[i].x;
            sum_y += cluster_cloud1[i].y;
            sum_z += cluster_cloud1[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud1.size();
        p_.y = sum_y / cluster_cloud1.size();
        p_.z = sum_z / cluster_cloud1.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
		sum_x = 0; sum_y = 0; sum_z = 0;
    }

	if (cluster_cloud2.size() != 0){
        for (size_t i=0; i<cluster_cloud2.size(); i++){
           // cout << cluster_cloud1[i].x << endl;
            sum_x += cluster_cloud2[i].x;
            sum_y += cluster_cloud2[i].y;
            sum_z += cluster_cloud2[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud2.size();
        p_.y = sum_y / cluster_cloud2.size();
        p_.z = sum_z / cluster_cloud2.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
		sum_x = 0; sum_y = 0; sum_z = 0;
    }

    if (cluster_cloud3.size() != 0){
        for (size_t i=0; i<cluster_cloud3.size(); i++){
           // cout << cluster_cloud2[i].x << endl;
            sum_x += cluster_cloud3[i].x;
            sum_y += cluster_cloud3[i].y;
            sum_z += cluster_cloud3[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud3.size();
        p_.y = sum_y / cluster_cloud3.size();
        p_.z = sum_z / cluster_cloud3.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
		sum_x = 0; sum_y = 0; sum_z = 0;
    }

    if (cluster_cloud4.size() != 0){
        for (size_t i=0; i<cluster_cloud4.size(); i++){
           // cout << cluster_cloud2[i].x << endl;
            sum_x += cluster_cloud4[i].x;
            sum_y += cluster_cloud4[i].y;
            sum_z += cluster_cloud4[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud4.size();
        p_.y = sum_y / cluster_cloud4.size();
        p_.z = sum_z / cluster_cloud4.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
    }

    //point_pub_.publish(mean_point);

	return filter(mean_p);


	///////////////////////////////////////////////////////////////

	// never happen below

    //point_pub_.publish(mean_point);

	Result_cloud += cluster_cloud1;
	Result_cloud += cluster_cloud2;
	Result_cloud += cluster_cloud3; 
	Result_cloud += cluster_cloud4; 
	//cout << mean_p.size() << endl;
	
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(Result_cloud, cloud_p);

    sensor_msgs::PointCloud2 result;
    pcl_conversions::fromPCL(cloud_p, result);
    result.header.frame_id = "velodyne";
    //pub_.publish(result);
}

vector<geometry_msgs::Point> Cluster::filter(vector<geometry_msgs::Point> points) { 
	points = sort(points);
//	points = sortFilter(points);

	if(points.size() > 4) {
		vector<geometry_msgs::Point> new_points;
		new_points.push_back(points.at(0));
		new_points.push_back(points.at(1));
		new_points.push_back(points.at(2));
		new_points.push_back(points.at(3));
		return new_points;
	} else {
		return points;
	}
}

vector<geometry_msgs::Point> Cluster::sort(vector<geometry_msgs::Point> list) {

  int i, j, least;
  int size = list.size();

  geometry_msgs::Point temp;

  for(i=0; i<size-1; i++){
    least = i;

    for(j=i+1; j<size; j++){
      if(getDist(list.at(j)) < getDist(list.at(least)))
        least = j;
    }

    if(i != least){
		temp = list.at(i);
		list.at(i) = list.at(least);
		list.at(least) = temp;
    }
  }

  return list;
}

vector<geometry_msgs::Point> Cluster::sortFilter(vector<geometry_msgs::Point> list) {

	vector<int> del_elements;

	for(int i=0;i<list.size()-1;i++) {
		if(abs(list.at(i).x-list.at(i+1).x) < 2.0) { // TODO: make it to parameter
			del_elements.push_back(i+1);
		}
	}

/*	for(int i=0;i<del_elements.size();i++) {
		list.erase(del_elements.at(i));
	}*/

	return list;
}

double Cluster::getDist(geometry_msgs::Point point) {
	return sqrt(pow(point.x-0, 2) + pow(point.y-0, 2));
}

