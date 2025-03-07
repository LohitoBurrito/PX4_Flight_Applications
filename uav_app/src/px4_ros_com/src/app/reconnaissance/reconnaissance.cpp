
/**
 * @brief UAV Command node
 * @file uav_app.cpp
 * @author Lohit Muralidharan
 */


#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdint.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class ReconNode : public rclcpp::Node
{
    public:

		// Node constructor
		explicit ReconNode();

		// Subscriber Callbacks
		void read_vehicle_attitude(const VehicleAttitude::UniquePtr msg);
		void read_local_position(const VehicleLocalPosition::UniquePtr msg);
		void read_camera_image_raw(const sensor_msgs::msg::Image::SharedPtr msg);

		// Publisher Callbacks
		void publish_offboard_control_mode();
		void publish_trajectory_setpoint(float x, float y, float z, float yaw);
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

		// Offboard Control Mode
		void arm();
		void disarm();

		// run mission
		void run_mission();

    private:

		enum direction { NORTH, SOUTH, WEST, EAST };

		typedef struct VehicleState {
			float x;
			float y;
			float z;
			float roll;
			float pitch;
			float yaw;
			float waypoint_x;
			float waypoint_y;
			float waypoint_z;
			float waypoint_yaw;
			direction dir;
			bool abort_mission;
		} VehicleState;

		std::array<float, 4> recon_bounds = { -55.0, 55.0, -55.0, 55.0 };

		rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber;
		rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscriber;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_raw_subscriber;

		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
		
		rclcpp::TimerBase::SharedPtr timer_;	
		std::atomic<uint64_t> timestamp_;   	//!< common synced timestamped
		uint64_t offboard_setpoint_counter_;    //!< counter for the number of setpoints sent

		VehicleState vehicle_state;
		float tolerarance = 0.75;
		float peak_altitude = -10.0;
		bool ms1 = false;
		bool ms2 = false;
		bool ms3 = false;
		bool ms4 = false;
		bool ms5 = false;

		cv::dnn::Net net;
		float thresh = 0.5;
		float nms_thresh = 0.45; 
		float marker_x;
		float marker_y;
};


/**
 * @brief UAV Command node constructor
 * @link 
 */
ReconNode::ReconNode() : Node("uav_command") {

	std::cout << "=============================Instantiating UAV Command Node=============================" << std::endl;

	net = cv::dnn::readNetFromONNX("src/px4_ros_com/src/app/reconnaissance/model/best.onnx");

	rmw_qos_profile_t qos_profile_deg = rmw_qos_profile_sensor_data;
	rmw_qos_profile_t qos_profile_pos = rmw_qos_profile_sensor_data;
	rmw_qos_profile_t qos_profile_camera = rmw_qos_profile_sensor_data;


	rclcpp::QoS qos_deg(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_deg);
	rclcpp::QoS qos_pos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_pos);
	rclcpp::QoS qos_camera(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_camera);

	vehicle_state.waypoint_x = 0;
	vehicle_state.waypoint_y = 0;
	vehicle_state.waypoint_z = 0;

	vehicle_attitude_subscriber = this->create_subscription<VehicleAttitude>(
		"/fmu/out/vehicle_attitude",
		qos_deg,
		std::bind(&ReconNode::read_vehicle_attitude, this, std::placeholders::_1)
	);

	local_position_subscriber = this->create_subscription<VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", 
		qos_pos,
		std::bind(&ReconNode::read_local_position, this, std::placeholders::_1)
	);

	camera_image_raw_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera/image_raw", 
		qos_camera,
		std::bind(&ReconNode::read_camera_image_raw, this, std::placeholders::_1)
	);

	offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);


	offboard_setpoint_counter_ = 0;
	
	timer_ = this->create_wall_timer(300ms, std::bind(&ReconNode::run_mission, this));

}

void ReconNode::read_vehicle_attitude(const VehicleAttitude::UniquePtr msg) {

}

/**
 * @brief UAV Command Local Position Subscriber Callback
 * @param msg Vehicle Local Position message pointer
 * @link 
 */
void ReconNode::read_local_position(const VehicleLocalPosition::UniquePtr msg) {
	vehicle_state.x = msg->x;
	vehicle_state.y = msg->y;
	vehicle_state.z = msg->z;
	vehicle_state.yaw = std::atan2(msg->vy, msg->vx);
	// std::cout << vehicle_state.x << " " << vehicle_state.y << " " << vehicle_state.z << std::endl;
}

/**
 * @brief UAV Command Camera Subscriber Callback
 * @param msg Camera Image Raw message pointer
 * @link 
 */
void ReconNode::read_camera_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {

	try {

		RCLCPP_INFO(this->get_logger(), "===============IMAGE RETRIEVED===============");
		cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

		cv::Mat resized_frame;
		cv::resize(frame, resized_frame, cv::Size(640, 640));

		cv::Mat blob = cv::dnn::blobFromImage(resized_frame, 1.0/255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);

		net.setInput(blob);

		cv::Mat output = net.forward();

		// Run Thresholding
		std::vector<cv::Rect> boxes;
    	std::vector<float> scores;

		for (int i = 0; i < output.size[1]; ++i) {
			float score = output.at<float>(0, i, 4);
			if (score > thresh) {
				int classId = output.at<float>(0, i, 5) > output.at<float>(0, i, 6) ? 5 : 6;
	
				float cx = output.at<float>(0, i, 0);
				float cy = output.at<float>(0, i, 1);
				float w = output.at<float>(0, i, 2);
				float h = output.at<float>(0, i, 3);

				int x = static_cast<int>(cx - w / 2);
    			int y = static_cast<int>(cy - h / 2);
	
				boxes.push_back(cv::Rect(x, y, static_cast<int>(w), static_cast<int>(h)));
				scores.push_back(output.at<float>(0, i, classId)); // Use class score
			}
		}
	
		// Run NMS
		std::vector<int> indices;
		cv::dnn::NMSBoxes(boxes, scores, thresh, nms_thresh, indices);

		if (indices.size() == 0)
			marker_x = marker_y = -1;

		for (int idx : indices) {

			int id = static_cast<int>(output.at<float>(0, idx, 5));
			std::string label = (id == 0) ? "car" : "empty";
			int label_x = boxes[idx].x;
			int label_y = boxes[idx].y - 10; 
			cv::putText(resized_frame, label, cv::Point(label_x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
			cv::rectangle(resized_frame, boxes[idx], cv::Scalar(0, 255, 0), 2);

			std::cout << "ID: " << id << std::endl;

			if (id == 0 && vehicle_state.x > recon_bounds[0] + 5.0  && vehicle_state.x < recon_bounds[1] - 5.0 && vehicle_state.y > recon_bounds[2] - 5.0 && vehicle_state.x < recon_bounds[3] - 5.0) {	

				ms1 = ms2 = ms3 = ms4 = true;
				marker_x = boxes[idx].x + boxes[idx].width  / 2;
				marker_y = boxes[idx].y + boxes[idx].height / 2;

				vehicle_state.waypoint_x = vehicle_state.x;
				vehicle_state.waypoint_y = vehicle_state.y;
				vehicle_state.waypoint_z = vehicle_state.z;

				publish_offboard_control_mode();
				publish_trajectory_setpoint(vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z, vehicle_state.waypoint_yaw);

				cv::imwrite("src/px4_ros_com/src/app/reconnaissance/images/caught_frame.jpg", resized_frame);

				break;
			}

		}

		// Debug output
		cv::imwrite("src/px4_ros_com/src/app/reconnaissance/images/cv_frame.jpg", resized_frame);

	} catch (cv_bridge::Exception& e) {

		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());

	}
}

/**
 * @brief UAV Command Offboard Control Publisher
 * @link 
 */
void ReconNode::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher->publish(msg);
}

/**
 * @brief UAV Command Offboard Trajectory Publisher
 * @param x x (m)
 * @param y y (m)
 * @param z altitude (m)
 * @param yaw yaw angle (rad)
 * @link 
 */
void ReconNode::publish_trajectory_setpoint(float x, float y, float z, float yaw) {
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher->publish(msg);
}

/**
 * @brief UAV Vehicle Command Publisher
 * @param command   Command code
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @link 
 */
void ReconNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher->publish(msg);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void ReconNode::arm() { 
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void ReconNode::disarm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief UAV Mission Checkpoints
 *	1. ms1 -> Arm the vehicle and set offboard control mode
 *	2. ms2 -> Get to coordinate (0m, 0m, -15m)
 * 	3. ms3 -> Get to coordinate (-50m, 50m, -15m)
 *	4. ms4 -> Run a reconnaissance mission by going through each coordinate in the grid until (50m, -50, -15m)
 *     unless you locate a red car
 *	5. ms5 -> Confirm target by hovering over vehicle
 *  6. ms6 -> Land
 */
void ReconNode::run_mission() {

	auto dist = [] (float x1, float y1, float z1, float x2, float y2, float z2) {
		return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
	};

	if (vehicle_state.abort_mission) {

		RCLCPP_INFO(this->get_logger(), "================MISSION ABORT================");
		
		vehicle_state.waypoint_x = 0.0;
		vehicle_state.waypoint_y = 0.0;
		vehicle_state.waypoint_z = 0.0;

		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

		if (vehicle_state.z >= 0) {
			disarm();
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION , 1);
		}

		
	} else if (!ms1) {

		RCLCPP_INFO(this->get_logger(), "==================MISSION 1==================");

		if (offboard_setpoint_counter_ == 10) {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			arm();
			vehicle_state.waypoint_z = peak_altitude;
			ms1 = true;
		}
	
		if (offboard_setpoint_counter_ < 11)
			offboard_setpoint_counter_++;

		
	} else if (!ms2) {

		RCLCPP_INFO(this->get_logger(), "==================MISSION 2==================");

		if (dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, 0.0, 0.0, peak_altitude) <= tolerarance) {
			vehicle_state.dir = EAST;
			vehicle_state.waypoint_x = recon_bounds[0];
			vehicle_state.waypoint_y = recon_bounds[3];
			vehicle_state.waypoint_z = peak_altitude;
			ms2 = true;
		}

		
	} else if (!ms3) {

		RCLCPP_INFO(this->get_logger(), "==================MISSION 3==================");

		if (dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, recon_bounds[0], recon_bounds[3], peak_altitude) <= tolerarance) {
			ms3 = true;
			vehicle_state.dir = WEST;
			vehicle_state.waypoint_x = recon_bounds[1];
			vehicle_state.waypoint_y = recon_bounds[3];
			vehicle_state.waypoint_z = peak_altitude;
		}
		vehicle_state.waypoint_yaw = vehicle_state.yaw;

		
	} else if (!ms4) {

		RCLCPP_INFO(this->get_logger(), "==================MISSION 4==================");

		if (vehicle_state.waypoint_y == recon_bounds[2] && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) < tolerarance) {
			vehicle_state.abort_mission = true;
		} else if ((vehicle_state.dir == EAST || vehicle_state.dir == WEST) && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) < tolerarance) {
			vehicle_state.dir = SOUTH;
			vehicle_state.waypoint_y -= 5.0;
		} else if (vehicle_state.dir == SOUTH && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) < tolerarance) {
			if (vehicle_state.waypoint_x == -50.0)
				vehicle_state.dir = WEST;
			else 
				vehicle_state.dir = EAST;

			vehicle_state.waypoint_x *= -1;
		}
		vehicle_state.waypoint_yaw = vehicle_state.yaw;

		
	} else if (!ms5) {

		RCLCPP_INFO(this->get_logger(), "==================MISSION 5==================");
		
		if ((marker_x >= 315 && marker_x <= 325) && (marker_y >= 315 && marker_y <= 325)) {

			ms5 = true;

		} else if (marker_x == -1 && marker_y == -1) {

			if (vehicle_state.dir == EAST) {
				vehicle_state.waypoint_x += 0.1;
				if (vehicle_state.waypoint_x > recon_bounds[1]) {
					vehicle_state.waypoint_x = recon_bounds[0];
					ms4 = false;
				}

			} else {
				vehicle_state.waypoint_x -= 0.1;
				if (vehicle_state.waypoint_x < recon_bounds[0]) {
					vehicle_state.waypoint_x = recon_bounds[1];
					ms4 = false;
				}
			}
			
		} else {

			float disp_x = (vehicle_state.dir == EAST) ? marker_y - 320 : 320 - marker_y;
			float disp_y = 320 - marker_x;

			float target_rad = std::atan2(disp_y, disp_x);
			float radius = (sqrt(pow(disp_x, 2) + pow(disp_y, 2)) < 220.0) ? 0.1 : 0.3;

			vehicle_state.waypoint_x = vehicle_state.x + radius * std::cos(target_rad);
			vehicle_state.waypoint_y = vehicle_state.y + radius * std::sin(target_rad);
			vehicle_state.waypoint_z = peak_altitude;

			std::cout << "Yaw: " << vehicle_state.waypoint_yaw << std::endl;
			std::cout << "Angle: " << std::atan2(disp_y, disp_x) << std::endl;
			std::cout << "Current X: " << vehicle_state.x << std::endl;
			std::cout << "Current Y: " << vehicle_state.y << std::endl;
			std::cout << "Delta X: " << vehicle_state.x + radius * std::cos(target_rad) << std::endl;
			std::cout << "Delta Y: " << vehicle_state.y + radius * std::sin(target_rad) << std::endl;
		}

	} else {

		RCLCPP_INFO(this->get_logger(), "===============MISSION SUCCESS===============");

		vehicle_state.waypoint_x = vehicle_state.x;
		vehicle_state.waypoint_y = vehicle_state.y;
		vehicle_state.waypoint_z = 0.0;

		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

		if (vehicle_state.z >= 0) {
			disarm();
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION , 1);
		}

	}

	publish_offboard_control_mode();
	publish_trajectory_setpoint(vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z, vehicle_state.waypoint_yaw);
	
}


int main(int argc, char *argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ReconNode>());

	rclcpp::shutdown();
	return 0;
}