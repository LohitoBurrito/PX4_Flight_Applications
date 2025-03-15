#include <template.hpp>


/**
* @brief UAV Command Global Position Subscriber Callback
* @param msg Vehicle Global Position message pointer
* @link 
*/
void UAVNode::read_gps(const VehicleGlobalPosition::UniquePtr msg) {
    sensor_msg.lon = msg->lon;
    sensor_msg.lat = msg->lat;
    sensor_msg.alt = msg->alt; 
    write_file();
}

/**
* @brief UAV Command Sensor Combined Subscriber Callback
* @param msg Vehicle Sensor Combined message pointer
* @link 
*/
void UAVNode::read_imu(const SensorCombined::UniquePtr msg) {
    float accel_x = msg->accelerometer_m_s2[0];
    float accel_y = msg->accelerometer_m_s2[1];
    float accel_z = msg->accelerometer_m_s2[2];

    sensor_msg.roll = std::atan2(accel_y, std::sqrt(std::pow(accel_x, 2) + std::pow(accel_z, 2)));
    sensor_msg.pitch = std::atan2(-accel_x, std::sqrt(std::pow(accel_y, 2) + std::pow(accel_z, 2)));
    // sensor_msg.yaw += msg->gyro_rad[2];    
    write_file();   
}

/**
* @brief UAV Command Local Position Subscriber Callback
* @param msg Vehicle Local Position message pointer
* @link 
*/
void UAVNode::read_local_position(const VehicleLocalPosition::UniquePtr msg) {
    vehicle_state.x = msg->x;
    vehicle_state.y = msg->y;
    vehicle_state.z = msg->z;
    vehicle_state.vx = msg->vx;
	vehicle_state.vy = msg->vy;
	vehicle_state.vz = msg->vz;
    vehicle_state.yaw = std::atan2(msg->vy, msg->vx);
    // std::cout << vehicle_state.x << " " << vehicle_state.y << " " << vehicle_state.z << std::endl;
}


/**
* @brief UAV Command Camera Threshold Filter
* @param frame output matrix from model
* @param boxes filtered boxes
* @param scores filtered scores
* @link 
*/
void UAVNode::run_camera_threshold(const cv::Mat& output, std::vector<cv::Rect>& boxes, std::vector<float>& scores) {
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
}

/**
* @brief UAV Command Camera Subscriber Callback
* @param msg Camera Image Raw message pointer
* @link 
*/
void UAVNode::read_camera_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        run_camera_threshold(output, boxes, scores);

        // Run NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, thresh, nms_thresh, indices);

        process_output(resized_frame, output, boxes, indices);

        // Debug output
        write_image(image_path, resized_frame);

    } catch (cv_bridge::Exception& e) {

        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());

    }
}

/**
* @brief UAV create subscribers
* @link 
*/
void UAVNode::create_subscribers() {

    RCLCPP_INFO(this->get_logger(), "==============SUBSCRIBERS BOOTED=============");


    rmw_qos_profile_t qos_profile_pos = rmw_qos_profile_sensor_data;
    rmw_qos_profile_t qos_profile_camera = rmw_qos_profile_sensor_data;

    rmw_qos_profile_t qos_profile_pos2 = rmw_qos_profile_sensor_data;
    rmw_qos_profile_t qos_profile_imu = rmw_qos_profile_sensor_data;

    rclcpp::QoS qos_pos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_pos);
    rclcpp::QoS qos_camera(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_camera);

    rclcpp::QoS qos_pos2(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_pos2);
    rclcpp::QoS qos_imu(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_imu);

    global_position_subscriber = this->create_subscription<VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", 
        qos_pos2,
        std::bind(&UAVNode::read_gps, this, std::placeholders::_1)
    );

    sensor_combined_subscriber = this->create_subscription<SensorCombined>(
        "/fmu/out/sensor_combined", 
        qos_imu,
        std::bind(&UAVNode::read_imu, this, std::placeholders::_1)
    );

    local_position_subscriber = this->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", 
        qos_pos,
        std::bind(&UAVNode::read_local_position, this, std::placeholders::_1)
    );

    camera_image_raw_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        qos_camera,
        std::bind(&UAVNode::read_camera_image_raw, this, std::placeholders::_1)
    );
}