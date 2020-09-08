#include "barrel_avoidance/barrel_avoidance.h"

void StaticAvoidance::initSetup() {
	// ros
    pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/output_points", 10);
    sub_ = nh_.subscribe("/velodyne_points", 10, &StaticAvoidance::pointCallback, this);

	// values
	status_ = AVOID_CLOSE;
	obs_count_ = 0;
	obs_status_ = -1;

	init_flag_ = false;
	obs_status_flag_ = false;
}

void StaticAvoidance::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input) {
	switch(status_) {
		case AVOID_FAR:
			obstacles_ = Cluster().cluster(input, 0.5, 5, -1.5, 1.5);
			break;
		default:
			obstacles_ = Cluster().cluster(input, 0.5, 10, -3.0, 3.0);
			break;
	}
}

void StaticAvoidance::run() {
	if(!init_flag_) { // sleep once after node starts
		ros::Duration(0.5).sleep();
		init_flag_ = true;
	}

	if(!obs_status_flag_) {
		if(obstacles_.size() == 2) {
			obs_status_ = (obstacles_.at(0).y > obstacles_.at(1).y) ? LEFT : RIGHT;
			obs_status_flag_ = true;
		} else {
			return;
		}
	}

	try {
		switch(status_) {
			case AVOID_CLOSE:
				avoidClose();
				break;
			case AVOID_FAR:
				avoidFar();
				break;
			case ESCAPE:
				escape();
				break;
			default:
				break;
		}
	} catch(const std::out_of_range& oor) {
		cout << endl;
		cout << "!!!!! out of range exception occured !!!!!" << endl;
		cout << "status -> " << status_ << endl;
		cout << endl;

		steer_ = 0;
		speed_ = DEFAULT_SPEED;
	}
	 
	go();
	printStatus();
}

void StaticAvoidance::avoidClose() {
	if(obstacles_.size() < 2) {
		if(obs_count_ > 3) {
			status_++;
			obs_count_ = 0;
			return;
		} else {
			obs_count_++;
			return;
		}
	}

	obs_count_ = 0;

	geometry_msgs::Point point;
	point.x = obstacles_.at(0).x;
	point.y = obstacles_.at(1).y; 

	visualize(point);
	visualize(obstacles_);

	steer_ = calcSteer(point) * 1.0;
	speed_ = DEFAULT_SPEED;
}

void StaticAvoidance::avoidFar() {
	if(getDist(obstacles_.at(0)) > 2.0 || obstacles_.at(0).x > 1.0) {
		obs_count_ = 0;

		geometry_msgs::Point point;
		point.x = obstacles_.at(0).x;
		point.y = obstacles_.at(0).y;

		visualize(point);
		visualize(obstacles_);

		cout << "dist -> " << getDist(obstacles_.at(0)) << endl;

		steer_ = calcSteer(point) * 1.0;
		speed_ = DEFAULT_SPEED;
	} else {
		status_++;
	}
}

void StaticAvoidance::escape() {
	//steer_ = (first_obs_dir_ == RIGHT) ? LEFT_ESCAPE_STEER : RIGHT_ESCAPE_STEER;
	steer_ = (obs_status_ == RIGHT) ? ESCAPE_LEFT_STEER : ESCAPE_RIGHT_STEER;
	speed_ = ESCAPE_SPEED;
}

void StaticAvoidance::go() {
	ackerData_.drive.steering_angle = steer_;
	ackerData_.drive.speed = speed_;
	pub_.publish(ackerData_);
}

void StaticAvoidance::fixObstacleStatus() {
}

void StaticAvoidance::printStatus() {
	cout << endl;
	cout << "------------------------------------------" << endl;
	cout << "status              -> " << status_ << endl;
	cout << "speed               -> " << speed_ << endl;
	cout << "steer               -> " << steer_ << endl;
	cout << "number of obstacles -> " << obstacles_.size() << endl;
	cout << "------------------------------------------" << endl;
}

double StaticAvoidance::calcAngle(geometry_msgs::Point p){
	return atan(p.y/p.x) * 180 / M_PI;
}

double StaticAvoidance::calcSteer(geometry_msgs::Point p){
	return -atan(p.y/p.x) * 180 / M_PI;
}

double StaticAvoidance::getDist(geometry_msgs::Point p){
	return sqrt(pow(p.x - 0, 2) + pow(p.y - 0, 2));
}

void StaticAvoidance::visualize(geometry_msgs::Point point) {

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

	points.points.push_back(point);

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "static_avoidance_vlp16");
    StaticAvoidance sa;

    while(ros::ok()) {
		sa.run();
		ros::spinOnce();
    }
}

