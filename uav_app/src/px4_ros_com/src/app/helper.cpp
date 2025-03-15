#include <template.hpp>


/**
* @brief Constructor
*/
UAVNode::UAVNode() : Node("UAVNode") {}

/**
* @brief Debug
*/
void UAVNode::print_data() {
    std::cout << "Longitude: " << sensor_msg.lon << std::endl;
    std::cout << "Latitude: " <<  sensor_msg.lat << std::endl;
    std::cout << "Altitude: " <<  sensor_msg.alt << std::endl;
    std::cout << "Roll: " <<  sensor_msg.roll << std::endl;
    std::cout << "Pitch: " <<  sensor_msg.pitch << std::endl;
    std::cout << "Yaw: " <<  sensor_msg.yaw << std::endl;
}

/**
* @brief Write to data file
*/
void UAVNode::write_file() {
    std::ofstream file(data_path);
    if (!file) {
        std::cerr << "Error opening the file!" << std::endl;
        return;
    }

    file << sensor_msg.roll << "\n";
    file << sensor_msg.pitch << "\n";
    file << sensor_msg.yaw << "\n";
    file << sensor_msg.lon << "\n";
    file << sensor_msg.lat << "\n";
    file << sensor_msg.alt << "\n";

    file.close();
    // print_data(); 
}

/**
* @brief Write to image file
*/
void UAVNode::write_image(std::string image_path, cv::Mat resized_frame) {
    cv::imwrite(image_path, resized_frame);
}