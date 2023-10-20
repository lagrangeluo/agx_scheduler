#ifndef _SERVER_HPP_
#define _SERVER_HPP_

//std include
#include <string>
#include <vector>
#include <cmath>
#include <unistd.h> 
// #include <cstdio>
// #include <stdint.h>

//ros include
#include <ros/ros.h>

//msg include

//headfile include

//user defined header files
#include <agx_scheduler/SchedulePath.h>

//free_fleet include
#include <free_fleet/Server.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotState.hpp>

using namespace free_fleet;
namespace AgileX{

class agx_server_node
{
  public:
    //default constructor
    agx_server_node();

  private:
    //ros and callbacks
    ros::Subscriber schedule_path_sub;
    void schedule_path_callback(const agx_scheduler::SchedulePath::ConstPtr& traj);
    float caculate_yaw(const agx_scheduler::Waypoint& in, const agx_scheduler::Waypoint& out);
    //free_fleet server
    Server::SharedPtr _server;
    ServerConfig _server_config;
    int task_id = 0;

    //dds config
    int _dds_domin_id;
    std::string _dds_path_request_topic;
};



} //namespace AgileX


#endif //_SERVER_HPP_