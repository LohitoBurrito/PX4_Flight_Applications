#include <template.hpp>

/**
* @brief UAV Command Offboard Control Publisher
* @link 
*/
void UAVNode::publish_offboard_control_mode(bool pos, bool vel) {
    OffboardControlMode msg{};
    msg.position = pos;
    msg.velocity = vel;
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
void UAVNode::publish_trajectory_setpoint(float x, float y, float z, float yaw) {
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
void UAVNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
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
void UAVNode::arm() { 
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
* @brief Send a command to Disarm the vehicle
*/
void UAVNode::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
* @brief UAV create publishers
* @link 
*/
void UAVNode::create_publishers() {
    RCLCPP_INFO(this->get_logger(), "==============Publishers BOOTED==============");
    offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
}