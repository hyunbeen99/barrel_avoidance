#include "barrel_avoidance/barrel_avoidance.h"

void StaticAvoidance::initSetup() {
    pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/lidar_ackermann", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/output_points", 10);
    sub_ = nh_.subscribe("/velodyne_points", 10, &StaticAvoidance::pointCallback, this);
    imu_sub_ = nh_.subscribe("/gx5/imu/data", 10, &StaticAvoidance::imuCallback, this);

    status_ = 0;
    init_yaw_ = 0;
    second_yaw_ = 0;
    yaw_degree_ = 0;
	
    get_first_imu = true;
    get_second_imu = true;
}

ros::NodeHandle StaticAvoidance::getNodeHandle() {
	return this->nh_;
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

    if (get_first_imu && status_ == 1) {
        init_yaw_ = yaw_degree_;
        get_first_imu = false;
    }

    if (get_second_imu && status_ == 2) {
        second_yaw_ = yaw_degree_;
        get_second_imu = false;
    }
}

void StaticAvoidance::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input) {

	center_point_ = Cluster().cluster(input, 0.0, 0.0 ,0.0, 0.0); 

	switch(status_) {
		case 0:
			center_point_ = Cluster().cluster(input, 1, 15, -1.0, 1.0); 
			break;

		case 1:
			center_point_ = Cluster().cluster(input, 1, 10, -1.5, 1.5); 
			break;
			
		case 2:
			center_point_ = Cluster().cluster(input, 1, 5, -1.5, 1.5); 
			break;

		default:
			return;
	}

	visualize(center_point_);
}

void StaticAvoidance::run() {

    cout << "STATUS :: " << status_ << endl;

	//seek two obstacles before start
	try{
		switch(status_) {
			case 0:
				fixObstacles();
				break;

			case 1: {
					cout << "DIFF = " << abs(init_yaw_ - yaw_degree_) << endl;
					geometry_msgs::Point goalPoint;

					if(center_point_.size() == 2) {
						
						goalPoint.x = center_point_.at(0).x;
						goalPoint.y = center_point_.at(1).y;

						ackerData_.drive.steering_angle = 1.2*calcSteer(goalPoint);
						ackerData_.drive.speed = SPEED1;

						if(abs(init_yaw_ - yaw_degree_) >= COMMON_YAW_DIFF){
							ackerData_.drive.steering_angle = calcSteer(center_point_.at(1));
						}

						dist = getDist(center_point_.at(1));

					}else if (center_point_.size() < 2){

						ackerData_.drive.steering_angle = calcSteer(center_point_.at(0));
						ackerData_.drive.speed = SPEED1;

						dist = getDist(center_point_.at(0));
					}

					if (dist < STANDARD_DIST) status_++;
					break;
				}

			case 2:
				ackerData_.drive.steering_angle = obstacleAlign_.steer;
				cout << "DIFF = " << second_yaw_ - yaw_degree_ << endl;

				if (abs(second_yaw_ - yaw_degree_) > obstacleAlign_.yaw_diff) {
					steer = obstacleAlign_.steer;
					status_++;
				}
				break;

			case 3:
				cout << "DIFF = " << second_yaw_ - yaw_degree_ << endl;
				ackerData_.drive.steering_angle = steer;
				ackerData_.drive.speed = SPEED3;

				nh_.setParam("/static_finish1", true);
				break;

			default :
				return;
		}

	} catch(const std::out_of_range& oor) { 
		ackerData_.drive.steering_angle = STEER; 
		ackerData_.drive.speed = SPEED;
		cout << "out of range!!!!! " << endl;
	}

	pub_.publish(ackerData_);
}

void StaticAvoidance::fixObstacles(){
	if (center_point_.size() == 2){

		fixed_point_ = center_point_;

		if (fixed_point_.at(0).y > fixed_point_.at(1).y) { 
			obstacleAlign_.setAlign(LEFT_FIRST);
		} else {
			obstacleAlign_.setAlign(RIGHT_FIRST);
		}
		
		status_++;

	}else if (center_point_.size() < 2){
		ackerData_.drive.steering_angle = STEER; 
		ackerData_.drive.speed = SPEED;
	} 
}

double StaticAvoidance::getDist(geometry_msgs::Point point_){
	return sqrt(pow(point_.x - 0, 2) + pow(point_.y - 0, 2));
}

//L - R +
double StaticAvoidance::calcSteer(geometry_msgs::Point point_){
	return -atan(point_.y/point_.x) * 180 / M_PI;
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

	int cur_state = -2;
	sa.getNodeHandle().getParam("/kuuve_state", cur_state); // cur_state is STATIC_OBS1 in kuuve control

    while(ros::ok()) {
		if (cur_state == 1){
			sa.run();
		}

		ros::spinOnce();
    }
}
