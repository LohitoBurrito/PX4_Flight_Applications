
/**
* @brief FireForce node
* @file fireforce.cpp
* @author Lohit Muralidharan
*/

#include <template.hpp>
#include <http_lib.hpp>
#include <waypoint_parser.hpp>
#include <curl/curl.h>
#include <memory>  
#include <stdint.h>
#include <chrono>
#include <iostream>


class HTTPPOSTServer : public rclcpp::Node {
    public:
        // Node constructor
        explicit HTTPPOSTServer();
        // HTTP Rest API
        void post_frame();

    private: 

        rclcpp::TimerBase::SharedPtr timer_;	
        std::atomic<uint64_t> timestamp_;   	//!< common synced timestamped
        uint64_t offboard_setpoint_counter_;    //!< counter for the number of setpoints sent

        HTTPExecutor http_executor;

};

HTTPPOSTServer::HTTPPOSTServer() : Node("HTTP_SERVER") {

    timer_ = this->create_wall_timer(300ms, std::bind(&HTTPPOSTServer::post_frame, this));
    
}

void HTTPPOSTServer::post_frame() {
    
    RCLCPP_INFO(this->get_logger(), "=================FRAME POSTED================");
    http_executor.http_post(
        "src/px4_ros_com/src/app/fireforce/images/cv_frame.jpg",
        "src/px4_ros_com/src/app/fireforce/env/.env",
        "Content-Type: image/jpg",
        "/storage/v1/object/UAVFrame/cv_frame.jpg"
    );

    RCLCPP_INFO(this->get_logger(), "=================SENSOR POSTED================");
    http_executor.http_post(
        "src/px4_ros_com/src/app/fireforce/data/data.txt",
        "src/px4_ros_com/src/app/fireforce/env/.env",
        "Content-Type: text/plain",
        "storage/v1/object/UAVFrame/sensor_msg.txt"
    );
}

class FireDetectNode : public UAVNode {
    public:
        // Node constructor
        explicit FireDetectNode();
        
        // Override
        void run_mission() override;
        void process_output(cv::Mat& resized_frame, const cv::Mat& output, const std::vector<cv::Rect>& boxes, const std::vector<int> indices) override;

    private:
        WaypointParser wp_parser;
        float peak_altitude = -40.0;
};

/**
* @brief FireDetectNode constructor
* @link 
*/
FireDetectNode::FireDetectNode() {

    std::cout << "==========================Instantiating FireForce Command Node==========================" << std::endl;

    model_path = "src/px4_ros_com/src/app/fireforce/model/best.onnx";
    image_path = "src/px4_ros_com/src/app/fireforce/images/cv_frame.jpg";
    data_path = "src/px4_ros_com/src/app/fireforce/data/data.txt";

    net = cv::dnn::readNetFromONNX(model_path);

    sensor_msg.roll = 0.0;
    sensor_msg.pitch = 0.0;
    sensor_msg.yaw = 0.0;
    sensor_msg.lon = 0.0;
    sensor_msg.lat = 0.0;
    sensor_msg.alt = 0.0;

    vehicle_state.waypoint_x = 0;
    vehicle_state.waypoint_y = 0;
    vehicle_state.waypoint_z = 0;
    vehicle_state.wp = 0;
    vehicle_state.state = UAV_INIT;

    create_subscribers();
    create_publishers();

    wp_parser.parse_waypoints("src/px4_ros_com/src/app/fireforce/waypoint/waypoint.txt");

    offboard_setpoint_counter_ = 0;
    
    timer_ = this->create_wall_timer(300ms, std::bind(&FireDetectNode::run_mission, this));
}

/**
* @brief Process Yolov5 output
* @param resized_frame frame that we will annotate on
* @param output output of yolov5 model
* @param boxes boxes related to indices
* @param indices indices of most accurate inferences
* @link 
*/
void FireDetectNode::process_output(cv::Mat& resized_frame, const cv::Mat& output, const std::vector<cv::Rect>& boxes, const std::vector<int> indices) {
    if (indices.size() == 0)
        marker_x = marker_y = -1;

    // Process Output
    for (int idx : indices) {

        int id = static_cast<int>(output.at<float>(0, idx, 5));
        std::string label = (id == 0) ? "fire" : "empty";
        int label_x = boxes[idx].x;
        int label_y = boxes[idx].y - 10; 
        cv::putText(resized_frame, label, cv::Point(label_x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(resized_frame, boxes[idx], cv::Scalar(0, 255, 0), 2);

        if (id == 0) {	

            marker_x = boxes[idx].x + boxes[idx].width  / 2;
            marker_y = boxes[idx].y + boxes[idx].height / 2;

            vehicle_state.wp = -1;
            vehicle_state.state = MISSION_COMPLETE;

            break;
        }
    }
}

/**
* @brief UAV Mission Checkpoints
*   1. [ms = -1] -> abort mission and land at coordinates (0m, 2m, 0m)
*	2. [ms = 0] -> Arm the vehicle and set offboard control mode
*	3. [ms = 1] -> Get to coordinate (0m, 0m, -15m)
* 	4. [ms = 2] -> 
*/
void FireDetectNode::run_mission() {

    auto dist = [] (float x1, float y1, float z1, float x2, float y2, float z2) {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
    };

    auto set_wp = [this] (uint32_t wp_idx) {
        std::array<float, 4> waypoint = this->wp_parser.get_waypoint(wp_idx);
        this->vehicle_state.waypoint_x = waypoint[0];
        this->vehicle_state.waypoint_y = waypoint[1];
        this->vehicle_state.waypoint_z = waypoint[2];
        this->vehicle_state.waypoint_yaw = waypoint[3];
    };

    std::string wp_str = "==================MISSION " + std::to_string(vehicle_state.wp + 1) + "==================";

    switch (vehicle_state.state) {

        case ABORT_MISSION:

            RCLCPP_INFO(this->get_logger(), "================MISSION ABORT================");
            
            set_wp(0);

            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

            if (vehicle_state.z >= 0) {
                disarm();
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1);
            }

            break;

        case MISSION_COMPLETE:

            RCLCPP_INFO(this->get_logger(), "================FIRE DETECTED================");

            if (vehicle_state.dir == UP) 
                vehicle_state.waypoint_x = vehicle_state.x + ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));
            else
                vehicle_state.waypoint_x = vehicle_state.x - ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));

            vehicle_state.waypoint_y = vehicle_state.y + ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));
			vehicle_state.waypoint_z = vehicle_state.z - ((320.0 - marker_y) / 320.0);

            break;

        case UAV_INIT:

            RCLCPP_INFO(this->get_logger(), wp_str.c_str());

            if (offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
            }

            set_wp(vehicle_state.wp + 1);

            if (offboard_setpoint_counter_ < 11)
                offboard_setpoint_counter_++;
            
            break;

        case RUN_MISSION:
            
            RCLCPP_INFO(this->get_logger(), wp_str.c_str());

            set_wp(vehicle_state.wp + 1);

            break;
        
        default:
            break;
            
    }

    if (vehicle_state.wp >= 0 && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) <= tolerarance)
        vehicle_state.wp++;
    
    if (vehicle_state.wp == 6)
        vehicle_state.wp = -2;

    if (vehicle_state.wp > 0)
        vehicle_state.state = RUN_MISSION;
    else if (vehicle_state.wp == -2)
        vehicle_state.state = ABORT_MISSION;
    else if (vehicle_state.wp == -1)
        vehicle_state.state = MISSION_COMPLETE;
    else
        vehicle_state.state = UAV_INIT;

    sensor_msg.yaw = vehicle_state.waypoint_yaw * 180.0 / pi;

    publish_offboard_control_mode(true, false);
    publish_trajectory_setpoint(vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z, vehicle_state.waypoint_yaw);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto uav_command = std::make_shared<FireDetectNode>();
    // auto http_server = std::make_shared<HTTPPOSTServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(uav_command);
    // executor.add_node(http_server);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}

