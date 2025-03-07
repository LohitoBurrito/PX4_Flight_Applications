
/**
* @brief FireForce node
* @file fireforce.cpp
* @author Lohit Muralidharan
*/

#include <template.h>
#include <curl/curl.h>
#include <memory>  
#include <stdint.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

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
        CURL* curl;
        CURLcode res;

        std::string supabase_url = ""; // TODO: Fill out information
        std::string key = "";	       // TODO: Fill out information
        std::string image_path;
        std::string data_path;

};

HTTPPOSTServer::HTTPPOSTServer() : Node("HTTP_SERVER") {

    image_path = "src/px4_ros_com/src/app/fireforce/images/cv_frame.jpg";
    data_path = "src/px4_ros_com/src/app/fireforce/data/data.txt";
    timer_ = this->create_wall_timer(300ms, std::bind(&HTTPPOSTServer::post_frame, this));
    
}

size_t read_callback(void *ptr, size_t size, size_t nmemb, void *data) {
    std::ifstream *file = static_cast<std::ifstream *>(data);
    file->read(reinterpret_cast<char *>(ptr), size * nmemb); 
    return file->gcount(); // Return the number of bytes read
}

void HTTPPOSTServer::post_frame() {
    
    RCLCPP_INFO(this->get_logger(), "=================FRAME POSTED================");
    curl = curl_easy_init();

    std::string auth = "Authorization: Bearer " + key;
    std::string type = "Content-Type: image/jpg";

    if (curl) {
        std::ifstream file(image_path, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
            return;
        }

        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, auth.c_str());
        headers = curl_slist_append(headers, type.c_str());

        std::string supabase_img_url = supabase_url + "/storage/v1/object/UAVFrame/cv_frame.jpg";
        curl_easy_setopt(curl, CURLOPT_URL, supabase_img_url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);
        curl_easy_setopt(curl, CURLOPT_READDATA, &file);
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK) 
            std::cerr << "POST REQUEST FAILED" << std::endl;

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
        file.close();
    }

    RCLCPP_INFO(this->get_logger(), "=================SENSOR POSTED================");
    curl = curl_easy_init();
    if (curl) {

        std::ifstream file(data_path);
        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
            return;
        }

        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, auth.c_str());
        headers = curl_slist_append(headers, type.c_str());

        std::string supabase_data_url = supabase_url + "storage/v1/object/UAVFrame/sensor_msg.txt";
        curl_easy_setopt(curl, CURLOPT_URL, supabase_data_url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);
        curl_easy_setopt(curl, CURLOPT_READDATA, &file);
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK) 
            std::cerr << "POST REQUEST FAILED" << std::endl;

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
        file.close();
    }
}



class FireDetectNode : public UAVNode {
    public:
        // Node constructor
        explicit FireDetectNode();
        //
        void run_mission() override;
};

/**
* @brief UAV Command node constructor
* @link 
*/
FireDetectNode::FireDetectNode() {

    std::cout << "=============================Instantiating UAV Command Node=============================" << std::endl;

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
    vehicle_state.ms = 0;

    create_subscribers();
    create_publishers();

    offboard_setpoint_counter_ = 0;
    
    timer_ = this->create_wall_timer(300ms, std::bind(&FireDetectNode::run_mission, this));
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

    switch (vehicle_state.ms) {

        case -2:

            RCLCPP_INFO(this->get_logger(), "================MISSION ABORT================");
            
            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 2.0;
            vehicle_state.waypoint_z = 0.0;

            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

            if (vehicle_state.z >= 0) {
                disarm();
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1);
            }

            break;

        case -1:

            RCLCPP_INFO(this->get_logger(), "================FIRE DETECTED================");

            if (vehicle_state.dir == UP) 
                vehicle_state.waypoint_x = vehicle_state.x + ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));
            else
                vehicle_state.waypoint_x = vehicle_state.x - ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));

            vehicle_state.waypoint_y = vehicle_state.y + ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));
			vehicle_state.waypoint_z = vehicle_state.z - ((320.0 - marker_y) / 320.0);

            break;

        case 0:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 1==================");

            if (offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
            }

            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 0.0;
            vehicle_state.waypoint_z = -5.0;
        
            if (offboard_setpoint_counter_ < 11)
                offboard_setpoint_counter_++;
            
            break;

        case 1:
            
            RCLCPP_INFO(this->get_logger(), "==================MISSION 2==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = 12.5;
            vehicle_state.waypoint_z = -5.0;
            vehicle_state.waypoint_yaw = -pi / 4;

            break;

        case 2:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 3==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = 12.5;
            vehicle_state.waypoint_z = peak_altitude;
            vehicle_state.waypoint_yaw = -pi / 4;
            vehicle_state.dir = UP;

            break;

        case 3:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 4==================");

            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 0.0;
            vehicle_state.waypoint_z = peak_altitude;
            vehicle_state.waypoint_yaw = 0.0;

            break;

        case 4:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 5==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = -12.5;
            vehicle_state.waypoint_z = peak_altitude;
            vehicle_state.waypoint_yaw = pi / 4;

            break;
        
        case 5:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 6==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = -12.5;
            vehicle_state.waypoint_z = -5.0;
            vehicle_state.waypoint_yaw = pi / 4;
            vehicle_state.dir = DOWN;

            break;

        default:
        
            break;
            
    }

    if (vehicle_state.ms >= 0 && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) <= tolerarance)
        vehicle_state.ms++;
    
    if (vehicle_state.ms == 6)
        vehicle_state.ms = -2;

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

