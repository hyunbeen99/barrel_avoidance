#include "static_avoidance/jinuk.h"

void StaticAvoidance::initSetup() {
	// pub
    pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/output_points", 10);

	// sub
    sub_ = nh_.subscribe("/velodyne_points", 10, &StaticAvoidance::pointCallback, this);
    imu_sub_ = nh_.subscribe("/gx5/imu/data", 10, &StaticAvoidance::imuCallback, this);

	// value
    status_ = 0;

	speed_ = 0;
	steer_ = 0;

    yaw_deg_ = 0;
    init_yaw_ = 0;

	init_flag_ = false;
}

void StaticAvoidance::imuCallback(const sensor_msgs::ImuConstPtr &imu){
	tf::Quaternion q(
			imu->orientation.x,
			imu->orientation.y,
			imu->orientation.z,
			imu->orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	yaw_deg_ = yaw*180/M_PI;

	// save initial yaw
    if (!init_flag_) {
        init_yaw_ = yaw_deg_;
        init_flag_ = true;
    }
}

void StaticAvoidance::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input) {
	obstacles_ = Cluster().cluster(input, 0.5, 10, -3.0, 3.0); 
}

void StaticAvoidance::run() {
	try{

	} catch(const std::out_of_range& oor){ 
		ackerData_.drive.steering_angle = 0.0; 
		ackerData_.drive.speed = 1.0;

		cout << "!!!!! out of range occured !!!!!" << endl;
	}
}

void go() {
	ackerData_.drive.steering_angle = steer_; 
	ackerData_.drive.speed = speed_;

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
