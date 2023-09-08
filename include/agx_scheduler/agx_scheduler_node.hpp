#ifndef AGX_SCHEDULER_HPP_
#define AGX_SCHEDULER_HPP_

//std include
#include <string>
#include <vector>
#include <chrono>
// #include <thread>
#include <unistd.h> 
#include <cstdio>
#include <stdint.h>
#include <queue>
#include <fstream>

//ros include
#include <ros/ros.h>

//msg include

//headfile include
//#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <rmf_traffic/geometry/Circle.hpp>
#include "parse_graph.hpp"
#include <yaml-cpp/yaml.h>

//user defined header files
#include <agx_scheduler/SchedulePath.h>
#include <agx_scheduler/Waypoint.h>
#include <agx_scheduler/add_waypoint.h>
#include <agx_scheduler/add_lane.h>

#define NULL_INDEX 65535

namespace AgileX{
class agx_scheduler_node
{

  public:
    //default constructor
    agx_scheduler_node();

    //init all search method
    bool init_search_method();
    
    //start greedy heuristic search using struct _goal and _start
    bool greedy_search_start();

    //read and set the value of struct _goal and _start
    bool set_goal_and_start(void);
    
    //start astar heuristic search using struct _goal and _start
    bool astar_search_start();
    std::size_t get_goal_index();
    std::size_t get_start_index();

    //the api for creating nav yaml file
    bool create_nav_yaml();
    class GreedyImplementation;
    class AStarImplementation;

    //the yaml node parser
    class GraphImplementation;
    bool add_waypoint_to_graph(Eigen::Vector2d location, std::string name, 
                                std::string floor_name,std::string nav_file_name);
    bool add_lanes_to_graph(std::size_t start_wp, std::size_t goal_wp,
                              std::string floor_name,std::string nav_file_name);

    //service callback function
    bool add_waypoint_callback(agx_scheduler::add_waypoint::Request& request, agx_scheduler::add_waypoint::Response& response);
    bool add_lane_callback(agx_scheduler::add_lane::Request& request, agx_scheduler::add_lane::Response& response);
  private:

    //ros
    ros::Publisher schedule_path_pub;
    ros::ServiceServer add_waypoint_server;
    ros::ServiceServer add_lane_server;
    ros::ServiceClient add_waypoint_client;

    struct Goal
    {
        Eigen::Vector2d location; 
        std::size_t index;
    };
    struct Start
    {
        Eigen::Vector2d location; 
        std::size_t index;
    };
    Goal _goal;
    Start _start;

    std::shared_ptr<rmf_traffic::agv::Graph> _graph;
    std::string _graph_file_path;
    std::string _config_path;

    std::shared_ptr<rmf_traffic::agv::VehicleTraits> _traits;
    std::shared_ptr<GreedyImplementation> _greedy_impl_ptr;
    std::shared_ptr<AStarImplementation> _astar_impl_ptr;
    std::shared_ptr<GraphImplementation> _graph_impl_ptr;
    
};
}//namespace AgileX

#endif