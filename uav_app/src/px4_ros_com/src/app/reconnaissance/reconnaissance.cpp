#include <template.hpp>

class ReconCommand : public UAVNode {
    public:
        explicit ReconCommand();
        void run_mission() override;
        void process_output(cv::Mat& resized_frame, const cv::Mat& output, const std::vector<cv::Rect>& boxes, const std::vector<int> indices) override;
    private:
        std::array<float, 4> recon_bounds = { -55.0, 55.0, -55.0, 55.0 };
        float peak_altitude = -10.0;
};

/**
* @brief ReconCommand constructor
* @link 
*/
ReconCommand::ReconCommand() {

    std::cout << "============================Instantiating Recon Command Node============================" << std::endl;

    model_path = "src/px4_ros_com/src/app/reconnaissance/model/best.onnx";
    image_path = "src/px4_ros_com/src/app/reconnaissance/images/cv_frame.jpg";
    data_path = "src/px4_ros_com/src/app/reconnaissance/data/data.txt";

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

    create_subscribers();
    create_publishers();

    offboard_setpoint_counter_ = 0;
    
    timer_ = this->create_wall_timer(300ms, std::bind(&ReconCommand::run_mission, this));
}

void ReconCommand::process_output(cv::Mat& resized_frame, const cv::Mat& output, const std::vector<cv::Rect>& boxes, const std::vector<int> indices) {
    if (indices.size() == 0)
			marker_x = marker_y = -1;

    for (int idx : indices) {

        int id = static_cast<int>(output.at<float>(0, idx, 5));
        std::string label = (id == 0) ? "car" : "empty";
        int label_x = boxes[idx].x;
        int label_y = boxes[idx].y - 10; 
        cv::putText(resized_frame, label, cv::Point(label_x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(resized_frame, boxes[idx], cv::Scalar(0, 255, 0), 2);

        if (id == 0 && vehicle_state.x > recon_bounds[0] + 5.0  && vehicle_state.x < recon_bounds[1] - 5.0 && vehicle_state.y > recon_bounds[2] - 5.0 && vehicle_state.x < recon_bounds[3] - 5.0) {	

            marker_x = boxes[idx].x + boxes[idx].width  / 2;
            marker_y = boxes[idx].y + boxes[idx].height / 2;
            vehicle_state.wp = 4;

            break;
        }

    }
}

void ReconCommand::run_mission() {

    auto dist = [] (float x1, float y1, float z1, float x2, float y2, float z2) {
		return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
	};

    switch (vehicle_state.wp) {
        case -1:

            RCLCPP_INFO(this->get_logger(), "================MISSION ABORT================");
            
            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 0.0;
            vehicle_state.waypoint_z = 0.0;

            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

            if (vehicle_state.z >= 0) {
                disarm();
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION , 1);
            }
            break;

        case 0:
            RCLCPP_INFO(this->get_logger(), "==================MISSION 1==================");

            if (offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
                vehicle_state.waypoint_z = peak_altitude;
                vehicle_state.wp++;
            }
        
            if (offboard_setpoint_counter_ < 11)
                offboard_setpoint_counter_++;
            break;

        case 1:
            RCLCPP_INFO(this->get_logger(), "==================MISSION 2==================");

            if (dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, 0.0, 0.0, peak_altitude) <= tolerarance) {
                vehicle_state.dir = EAST;
                vehicle_state.waypoint_x = recon_bounds[0];
                vehicle_state.waypoint_y = recon_bounds[3];
                vehicle_state.waypoint_z = peak_altitude;
                vehicle_state.wp++;
            }
            break;

        case 2:
            RCLCPP_INFO(this->get_logger(), "==================MISSION 3==================");

            if (dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, recon_bounds[0], recon_bounds[3], peak_altitude) <= tolerarance) {
                vehicle_state.dir = WEST;
                vehicle_state.waypoint_x = recon_bounds[1];
                vehicle_state.waypoint_y = recon_bounds[3];
                vehicle_state.waypoint_z = peak_altitude;
                vehicle_state.wp++;
            }
            vehicle_state.waypoint_yaw = vehicle_state.yaw;
            break;

        case 3:
            if (vehicle_state.waypoint_y == recon_bounds[2] && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) < tolerarance) {
                vehicle_state.wp = -1;
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
            break;

        case 4:
            RCLCPP_INFO(this->get_logger(), "==================MISSION 5==================");
            
            if ((marker_x >= 315 && marker_x <= 325) && (marker_y >= 315 && marker_y <= 325)) {

                vehicle_state.wp++;

            } else if (marker_x == -1 && marker_y == -1) {

                if (vehicle_state.dir == EAST) {
                    vehicle_state.waypoint_x += 0.1;
                    if (vehicle_state.waypoint_x > recon_bounds[1]) {
                        vehicle_state.waypoint_x = recon_bounds[0];
                        vehicle_state.wp--;
                    }

                } else {
                    vehicle_state.waypoint_x -= 0.1;
                    if (vehicle_state.waypoint_x < recon_bounds[0]) {
                        vehicle_state.waypoint_x = recon_bounds[1];
                        vehicle_state.wp--;
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
            break;

        case 5:
            RCLCPP_INFO(this->get_logger(), "===============MISSION SUCCESS===============");

            vehicle_state.waypoint_x = vehicle_state.x;
            vehicle_state.waypoint_y = vehicle_state.y;
            vehicle_state.waypoint_z = 0.0;

            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

            if (vehicle_state.z >= 0) {
                disarm();
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION , 1);
            }
            break;
        
        default:
            break;
    }

    publish_offboard_control_mode(true, false);
	publish_trajectory_setpoint(vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z, vehicle_state.waypoint_yaw);
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto uav_command = std::make_shared<ReconCommand>();
    // auto http_server = std::make_shared<HTTPPOSTServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(uav_command);
    // executor.add_node(http_server);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}