#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#include <string>

class WaypointParser {

    public:

        typedef struct Waypoint {
            float x;
            float y;
            float z;
            float yaw;

            Waypoint(float _x, float _y, float _z, float _yaw) : x(_x), y(_y), z(_z), yaw(_yaw) {}

        } Waypoint;


        WaypointParser();
        uint32_t parse_waypoints(std::string fp);
        std::array<float, 4> get_waypoint(uint32_t idx);

    private:

        std::string fp;
        std::vector<Waypoint> waypoints;

};

WaypointParser::WaypointParser() {}

uint32_t WaypointParser::parse_waypoints(std::string fp) {
    
    std::ifstream file(fp);
    if (!file.is_open()) {
        std::cerr << "Could not open Waypoint File" << std::endl;
        return 0;
    }

    std::string waypoint;
    while (std::getline(file, waypoint)) {

        std::stringstream ss(waypoint);

        float x, y, z, yaw;
        ss >> x >> y >> z >> yaw;

        waypoints.emplace_back(x, y, z, yaw);

    }

    return 1;
}

std::array<float, 4> WaypointParser::get_waypoint(uint32_t idx) {

    if (idx >= waypoints.size())
        return { 0.0, 0.0, 0.0, 0.0 };

    Waypoint waypoint = waypoints[idx];

    return {
        waypoint.x,
        waypoint.y,
        waypoint.z,
        waypoint.yaw
    };
}
