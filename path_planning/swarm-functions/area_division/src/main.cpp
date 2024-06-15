#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include <string>
#include <limits>

// Global variables to hold map dimensions and data
std::vector<signed char> grid;
int map_width = 0, map_height = 0;
float resolution = 0.1;

nav_msgs::OccupancyGrid received_map;

// Callback function prototype
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_width = msg->info.width;
    map_height = msg->info.height;
    resolution = msg->info.resolution;
    grid.assign(msg->data.begin(), msg->data.end());

    received_map.info.width = map_width;
    received_map.info.height = map_height;
    received_map.info.resolution = resolution;  // Each cell represents 1x1 meter
    received_map.data = msg->data;
    ROS_INFO_STREAM("Received map with width: " << map_width << ", height: " << map_height);
    std::ostringstream map_values;
    // map_values << "Map values: ";
    // for (size_t i = 0; i < grid.size() && i < 995; ++i) {
    //     for (size_t j = 0; j<grid.size()&& i < 995; ++j){
    //         map_values << static_cast<int>(grid[i,j]) << " ";
    //     }
    // }
    // ROS_INFO_STREAM(map_values.str());
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    // Subscriber for the map topic
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map_server", 10, mapCallback);

    area_division ad;

    // Wait for the first map to be received
    while (ros::ok() && (map_width == 0 || map_height == 0)) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();  // Sleep to wait for the map
    }
    ROS_INFO("area");
    // Now you have the map data, initialize your application
    // float resolution = 0.1;  // Assuming each grid cell is 1x1 meter
    ad.initialize_map(map_height, map_width, grid);
    int num_robots = 2;  // Set to 2 as per the logs indicating two robots

    std::map<std::string, std::vector<int>> cps_positions = {
        {"robot0", {18, 31-14}},
        {"robot1", {32, 31-12}} 
        // {"robot2", {1, 1}}  // Uncomment and add if needed
    };
    ROS_INFO("initPos");
    ad.initialize_cps(cps_positions);
    ROS_INFO("Pos int");
    // Access the coordinates of the robots
    std::map<std::string, std::vector<int>> coordinates;
    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        coordinates[robot_name] = cps_positions[robot_name];
    }

    // Create geometry_msgs::Point messages for starting positions
    std::map<std::string, geometry_msgs::Point> points;
    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);
        geometry_msgs::Point point;
        point.x = coordinates[robot_name][0] * resolution;
        point.y = coordinates[robot_name][1] * resolution;
        points[robot_name] = point;
    }

    ROS_INFO_STREAM("Map height: " << map_height << ", Map width: " << map_width << ", Grid size: " << grid.size());

    // Perform area division
    ad.divide();
    ROS_INFO("Division");
    // nav_msgs::OccupancyGrid map;
    // map.info.width = map_width;
    // map.info.height = map_height;
    // map.info.resolution = resolution;  // Each cell represents 1x1 meter
    // map.data = grid;

    // Publishers for the divided maps
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;
    std::map<std::string, nav_msgs::OccupancyGrid> robot_grids;

    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i);

        nav_msgs::OccupancyGrid robot_grid = ad.get_grid(received_map, robot_name);
        ros::Publisher robot_pub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "_grid", 10);
        ros::Publisher start_pos_pub = nh.advertise<geometry_msgs::Point>(robot_name + "_starting_pos", 1);

        robot_pubs[robot_name] = robot_pub;
        start_pos_pubs[robot_name] = start_pos_pub;
        robot_grids[robot_name] = robot_grid;

        ROS_INFO_STREAM("Initialized publishers for " << robot_name);
    }

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        for (int i = 0; i < num_robots; i++) {
            std::string robot_name = "robot" + std::to_string(i);
            std::string robot_grid_name = robot_name + "_grid";

            if (robot_grids.find(robot_name) != robot_grids.end()) {
                robot_pubs[robot_name].publish(robot_grids[robot_name]);
            } else {
                ROS_WARN_STREAM("Grid for " << robot_name << " not found.");
            }
            
            if (points.find(robot_name) != points.end()) {
                start_pos_pubs[robot_name].publish(points[robot_name]);
            } else {
                ROS_WARN_STREAM("Starting point for " << robot_name << " not found.");
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}