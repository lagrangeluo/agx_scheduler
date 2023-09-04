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

//ros include
#include <ros/ros.h>

//msg include

//headfile include
//#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <rmf_traffic/geometry/Circle.hpp>
#include "parse_graph.hpp"

//user defined header files
#include <agx_scheduler/SchedulePath.h>
#include <agx_scheduler/Waypoint.h>


#define NULL_INDEX 65535

namespace AgileX{
class agx_scheduler_node
{

  public:
    //default constructor
    agx_scheduler_node();

    //start greedy heuristic search using struct _goal and _start
    bool greedy_search_start();

    //read and set the value of struct _goal and _start
    bool set_goal_and_start(void);
    
    //start astar heuristic search using struct _goal and _start
    bool astar_search_start();
    std::size_t get_goal_index();
    std::size_t get_start_index();

    class GreedyImplementation;
    class AStarImplementation;

  private:

    //ros
    ros::Publisher schedule_path_pub;

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

    std::shared_ptr<rmf_traffic::agv::VehicleTraits> _traits;
    std::shared_ptr<GreedyImplementation> _greedy_impl_ptr;
    std::shared_ptr<AStarImplementation> _astar_impl_ptr;
    
};
}//namespace AgileX

#endif