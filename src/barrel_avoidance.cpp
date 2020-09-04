#include "static_avoidance/static_avoidance_hb.h"

void StaticAvoidance::initSetup() {
    pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/output_points", 10);
    sub_ = nh_.subscribe("/velodyne_points", 10, &StaticAvoidance::pointCallback, this);
    imu_sub_ = nh_.subscribe("/gx5/imu/data", 10, &StaticAvoidance::imuCallback, this);

    status_ = 0;
    init_yaw_ = 0;
    yaw_degree_ = 0;
    get_init_imu = false;

}

void StaticAvoidance::imuCallback(const sensor_msgs::ImuConstPtr &imu){
	tf::Quaternion q(
			imu->orientation.x,
			imu->orientation.y,
			imu->orientation.z,
			imu->orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	yaw_degree_ = yaw*180/M_PI;

	// get inital degree once at status 1
    if (get_init_imu == true) {
        init_yaw_ = yaw_degree_;
        get_init_imu = false;
    }
}

void StaticAvoidance::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input) {


    if (status_ == 0){
        center_point_ = Cluster().cluster(input, 1, 15, -2.5, 2.5); 
		print(center_point_);
        if (center_point_.size() >= 1) status_++;
		visualize(center_point_);
    }
	else if (status_ == 1){
		center_point_ = Cluster().cluster(input, 1, 10, -2, 2); 
		print(center_point_);
		visualize(center_point_);
	}
	else if (status_ == 2){
		center_point_ = Cluster().cluster(input, 1, 8, -1, 3); 
		print(center_point_);
		visualize(center_point_);
	}

}

void StaticAvoidance::run() {

//	cout << "###########################################" << endl;
//	cout << "STATUS :: " << status_ << endl;

	//seek two obstacles before start
	try{
		if (status_ == 1) {
			dist = getDist(center_point_.at(1));

			ackerData_.drive.steering_angle = calcSteer(center_point_.at(1));
			ackerData_.drive.speed = 1.2;

//		    cout << "DIST : " << dist << endl;

			if (dist < STANDARD_DIST) status_++;

		}   

		//seek only one obstacle
		else if (status_ == 2){

			get_init_imu = true; 

//			cout << "INIT YAW           : " << init_yaw_ << endl; 

			geometry_msgs::Point goalPoint_;

			goalPoint_.x = center_point_.at(0).x; 
			goalPoint_.y = center_point_.at(0).y + 1.5;

//			cout << "GOALPOINT_X         : " << goalPoint_.x << endl;
//			cout << "GOALPOINT_Y         : " << goalPoint_.y << endl;

			visualize(goalPoint_);

			ackerData_.drive.steering_angle = calcSteer(goalPoint_);
			ackerData_.drive.speed = 2.0;

			if (abs(init_yaw_ - yaw_degree_) >= 10) {
				ackerData_.drive.steering_angle = 0.5*(abs(init_yaw_ - yaw_degree_));
				ackerData_.drive.speed = 1.2;
			}

		}

//		cout << "###########################################" << endl;
//		cout << endl;

	} catch(const std::out_of_range& oor){ 
		ackerData_.drive.steering_angle = 0.0; 
		ackerData_.drive.speed = 1.2;

		cout << "No Obstacles" << endl;
	}

	pub_.publish(ackerData_);
}

double StaticAvoidance::getDist(geometry_msgs::Point point_){
	return sqrt(pow(point_.x - 0, 2) + pow(point_.y - 0, 2));
}

//L - R +
double StaticAvoidance::calcSteer(geometry_msgs::Point point_){
	return -atan(point_.y/point_.x) * 180 / M_PI;
}

void StaticAvoidance::visualize(geometry_msgs::Point point) {

	visualization_msgs::Marker points;
    
	points.header.frame_id = "velodyne";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;
	points.color.b = 1.0f;

	geometry_msgs::Point p;

	p.x = point.x;
	p.y = point.y;
	p.z = point.z;
	points.points.push_back(p);

	marker_pub_.publish(points);
}

void StaticAvoidance::visualize(vector<geometry_msgs::Point> input_points) {

	visualization_msgs::Marker points;

	points.header.frame_id = "velodyne";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 1;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;
	points.color.g = 1.0f;

	geometry_msgs::Point p;

	for (auto point : input_points) {
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		points.points.push_back(p);
	}

	marker_pub_.publish(points);
}

void StaticAvoidance::print(vector<geometry_msgs::Point> points){
	cout << "##################" << endl;

	try {
		cout << "0.X : " << points.at(0).x << endl;
		cout << "0.Y : " << points.at(0).y << endl;
		cout << "1.X : " << points.at(1).x  << endl;
		cout << "1.Y : " << points.at(1).y  << endl;
	} catch(const std::out_of_range& oor){ 
	}

	cout << "##################" << endl;
	cout << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "static_avoidance_vlp16");
    StaticAvoidance sa;

    while(ros::ok()) {
		sa.run();
		ros::spinOnce();
    }
}
