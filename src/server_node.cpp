#include "agx_scheduler/server_node.hpp"

using namespace AgileX;

agx_server_node::agx_server_node()
{
  //init ros
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  //init ros parameters
  private_nh.param<int>("dds_domain",_dds_domin_id, 42);
  private_nh.param<std::string>("dds_path_request_topic",_dds_path_request_topic,"path_request");

  schedule_path_sub = nh.subscribe<agx_scheduler::SchedulePath>("/agx_scheduler_node/schedule_path",10,
                            &agx_server_node::schedule_path_callback, this);

  //init free_fleet config
  _server_config.dds_domain = _dds_domin_id;
  _server_config.dds_path_request_topic = _dds_path_request_topic;

  //init the free fleet server
  Server::SharedPtr server = Server::make(_server_config);
  if (!server)
    ROS_ERROR("free fleet server init error!");
  _server = std::move(server);

}

void agx_server_node::schedule_path_callback(const agx_scheduler::SchedulePath::ConstPtr& traj)
{
  //
  uint16_t waypoint_num = traj->path.size();
  ROS_INFO("agx_server_node: traj has %d waypoints,start free fleet server",waypoint_num);
  
  //start wrap the path
  messages::PathRequest ff_msg;
  ff_msg.fleet_name = "agx_fleet";
  ff_msg.robot_name = "agx_fleet";
  ff_msg.task_id = "agx-server-node-" + std::to_string(task_id);
  task_id++;

  auto first = traj->path.begin();
  auto end = traj->path.end();
  while (first != end)
  {
    ros::Time time = ros::Time::now();
    messages::Location location;
    location.sec = time.sec;
    location.nanosec = time.nsec;
    location.x = (*first).location_x;
    location.y = (*first).location_y;
    //TODO: add yaw support
    location.yaw = 0;
    location.level_name = "L1";

    ff_msg.path.push_back(location);

    ++first;
  }
  _server->send_path_request(ff_msg);
}

int main(int argc, char * argv[])
{
  ROS_INFO("--------------agx_server_node--------------");
  ros::init(argc, argv, "/agx_server_node");

  ros::NodeHandle nh;


  agx_server_node agx_node;


  ros::spin();
  return 0;
}