
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <curl/curl.h>
#include <memory>  
#include <stdint.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class UAVNode : public rclcpp::Node {
    public:

		// Node Constructor
        explicit UAVNode();
        void create_subscribers();
        void create_publishers();

        // Subscriber Callbacks
        void read_gps(const VehicleGlobalPosition::UniquePtr msg);
        void read_imu(const SensorCombined::UniquePtr msg);
        void read_local_position(const VehicleLocalPosition::UniquePtr msg);
        void read_camera_image_raw(const sensor_msgs::msg::Image::SharedPtr msg);
        void run_camera_threshold(const cv::Mat& output, std::vector<cv::Rect>& boxes, std::vector<float>& scores);
        virtual void process_output(cv::Mat& resized_frame, const cv::Mat& output, const std::vector<cv::Rect>& boxes, const std::vector<int> indices) = 0;

        // Publisher Callbacks
		void publish_offboard_control_mode(bool pos, bool vel);
		void publish_trajectory_setpoint(float x, float y, float z, float yaw);
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

		// Offboard Control Mode
		void arm();
		void disarm();

        // Helpers
        void print_data();
        void write_file();
        void write_image(std::string image_path, cv::Mat resized_frame);

        // Mission 
        virtual void run_mission() = 0;

    protected:
        enum direction { NORTH, SOUTH, WEST, EAST, UP, DOWN };
        enum uav_state { ABORT_MISSION, MISSION_COMPLETE, UAV_INIT, RUN_MISSION };

        typedef struct VehicleState {
            float x;
            float y;
            float z;
            float roll;
            float pitch;
            float yaw;
            float vx;
            float vy;
            float vz;
            float waypoint_x;
            float waypoint_y;
            float waypoint_z;
            float waypoint_yaw;
            direction dir;
            int64_t wp;
            uav_state state;
        } VehicleState;

        typedef struct SensorMsg {
            float roll;
            float pitch;
            float yaw;
            float lon;
            float lat;
            float alt;
        } SensorMsg;

        rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_position_subscriber;
        rclcpp::Subscription<SensorCombined>::SharedPtr sensor_combined_subscriber;
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
        float pi = 3.14;

        SensorMsg sensor_msg;
        std::string data_path;

        cv::dnn::Net net;
        std::string model_path;
        std::string image_path;
        float thresh = 0.5;
        float nms_thresh = 0.45; 
        float marker_x;
        float marker_y;
};