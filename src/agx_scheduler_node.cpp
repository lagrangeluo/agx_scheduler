#include "agx_scheduler/agx_scheduler_node.hpp"

// namespace AgileX{


using namespace AgileX;

class agx_scheduler_node::GraphImplementation
{
  public:
  GraphImplementation(std::string nav_path,std::string config_path)
    :_nav_file_path(nav_path),_config_path(config_path)
  {
    //
  }

  std::shared_ptr<rmf_traffic::agv::Graph> make_graph()
  {
    //init
    auto traits = rmf_traffic::agv::VehicleTraits{
      {0.7, 0.5},
      {0.3, 1.5},
      rmf_traffic::Profile{
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(0.5),
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(1.5)
      }
    };

    traits.get_differential()->set_reversible(true);

    std::shared_ptr<rmf_traffic::agv::Graph> ptr = 
      std::make_shared<rmf_traffic::agv::Graph>(agx_scheduler::parse_graph(_nav_file_path, traits));
    
    //init graph
    ROS_INFO("graph parse complete,waypoint num: %ld, lane num: %ld",ptr->num_waypoints(),ptr->num_lanes());
    if(ptr->num_waypoints()<1)
      ROS_ERROR("waypoint num is zero!");

    return ptr;
  }

//  bool add_waypoint_to_graph(double& x,double& y, std::string& name)
  bool add_waypoint_to_graph(Eigen::Vector2d location, std::string name, 
                                std::string floor_name,std::string nav_file_name)
  {
    std::string file_path = _config_path + nav_file_name + ".yaml";
    
    //judge if the file exsis
    std::fstream file;
    file.open(file_path);
    if(!file)
    {
      ROS_WARN("ADD Waypoint: file not exist,create a new one!");
      std::ofstream file_create(file_path,std::fstream::out);
        if(file_create)
          ROS_INFO("ADD Waypoint: create new file success!");
        else
          ROS_ERROR("ADD Waypoint: create new file failed!");
      file.open(file_path);
        if(file)
          ROS_INFO("ADD Waypoint: reopen success!");
        else
          ROS_ERROR("ADD Waypoint: reopen faile!");
    }
    else
      ROS_INFO("ADD Waypoint: file open success!");

    //init graph node
    YAML::Node graph_config = YAML::LoadFile(file_path);
    if (!graph_config)
    {
      throw std::runtime_error("ADD Waypoint: Failed to load graph file [" + file_path + "]");
    }

    //wrap a waypoint node
    std::string property;
    if(!name.empty())
      property = "{name: "+ name + "}";
    else
      property = "{}";

    YAML::Node node_property = YAML::Load(property);
    YAML::Node waypoint_node;

    waypoint_node.push_back(location[0]);
    waypoint_node.push_back(location[1]);
    waypoint_node.push_back(node_property);

    //add the waypoint into graph node
    if(!graph_config.IsMap())
    { 
      //for empty graph node, we need to create all yaml node
      ROS_WARN("ADD Waypoint: the graph node is empty");
      graph_config["building_name"] = "agx_scheduler";

      YAML::Node vertices_node,Level_node;
      //vertex node
      vertices_node["vertices"].push_back(waypoint_node);
      //floor node
      Level_node[floor_name] = vertices_node;
      //level node
      graph_config["levels"] = Level_node;
      //record the waypoint num
      _waypoint_num = vertices_node["vertices"].size();
    }
    else
    {
      //we push back this node to the old one
      ROS_INFO("ADD Waypoint: the graph node has previous nodes");
      YAML::Node level_node = graph_config["levels"];
      YAML::Node floor_node = level_node[floor_name];
      if(floor_node.IsMap())
        ROS_INFO("ADD Waypoint: the floor is existed");
      else
        ROS_WARN("ADD Waypoint: the floor is not exist,we create the node");

      for(YAML::iterator it=floor_node["vertices"].begin();it!=floor_node["vertices"].end();++it)
      {
        const YAML::Node& waypoint = *it;
        if(waypoint[0].as<double>() == location[0] && waypoint[1].as<double>() == location[1])
        {
          ROS_ERROR("ADD Waypoint: the current wp has same location with a wp in graph node! Abort this request!");
          return false;
        }
      }
      floor_node["vertices"].push_back(waypoint_node);
      //record the waypoint num
      _waypoint_num = floor_node["vertices"].size();
    }

    // save the file
    std::ofstream file_writer(file_path, std::ios_base::out);
    //std::cout<< graph_config<<std::endl;
    file<<graph_config<<std::endl;

    //close the file
    file_writer.close();
    file.close();
    return true;
  }

  bool add_lanes_to_graph(std::size_t start_wp, std::size_t goal_wp,
                              std::string floor_name,std::string nav_file_name)
  {
    //
    std::string file_path = _config_path + nav_file_name + ".yaml";
    
    //judge if the file exsis
    std::fstream file;
    file.open(file_path);
    if(!file)
    {
      ROS_ERROR("ADD Lane: file not exist, for add lane operation it is forbidden");
      return false;
    }
    else
      ROS_INFO("ADD Lane: file open success!");

    //init yaml node
    YAML::Node graph_config = YAML::LoadFile(file_path);
    if (!graph_config)
    {
      throw std::runtime_error("ADD Lane: Failed to load graph file [" + file_path + "]");
    }

    //wrap a lane node
    YAML::Node lane_property = YAML::Load("{speed_limit: 0}");
    YAML::Node lane_node;
    lane_node.push_back(start_wp);
    lane_node.push_back(goal_wp);
    lane_node.push_back(lane_property);

    //add the lane into graph node
    bool floor_exist_flag = false;
    YAML::Node level_node = graph_config["levels"];
    for(auto iter=level_node.begin(); iter!=level_node.end(); iter++)
    {
      if(iter->first.as<std::string>() == floor_name)
      {
        //change the flag
        floor_exist_flag = true;
        ROS_INFO("ADD Lane: find level: %s",floor_name.c_str());
        //check if the waypoints of lane exist
        YAML::Node waypoint = iter->second["vertices"];
        if(start_wp > waypoint.size()-1 || goal_wp > waypoint.size()-1)
        {
          ROS_ERROR("ADD Lane: the waypoint of lanes does not exist! Abort this request");
          return false;
        }
        
        YAML::Node lanes = iter->second["lanes"];
        //check if it is a new lane
        for(auto iter = lanes.begin(); iter != lanes.end(); iter++)
        {
          YAML::Node lane = *iter;
          if(lane[0].as<std::size_t>() == start_wp && lane[1].as<std::size_t>() == goal_wp)
          {
            ROS_ERROR("ADD Lane: the current lane has same index with a lane in graph node! Abort the request!");
            return false;
          }
        }
        lanes.push_back(lane_node);
        //record the lane num
        _lane_num = lanes.size();
      }
    }
    if(floor_exist_flag == false)
    {
      ROS_ERROR("ADD Lane: floor %s is not exist!Abort the request!",floor_name.c_str());
      return false;
    }

    //clear the file will be written
    std::ofstream file_writer(file_path, std::ios_base::out);
    //std::cout<< graph_config<<std::endl;
    // save the file
    file<<graph_config<<std::endl;

    //close the file
    file_writer.close();
    file.close();
    return true;
  }

  bool delete_lane_of_graph(std::size_t start_wp, std::size_t goal_wp,
                               std::string floor_name, std::string nav_file_name)
  {
    std::string file_path = _config_path + nav_file_name + ".yaml";
    
    //judge if the file exsis
    std::fstream file;
    file.open(file_path);
    if(!file)
    {
      ROS_ERROR("DELETE Lane: file not exist, for delete lane operation it is forbidden");
      return false;
    }
    else
      ROS_INFO("DELETE Lane: file open success!");

    //init yaml node
    YAML::Node graph_config = YAML::LoadFile(file_path);
    if (!graph_config)
    {
      throw std::runtime_error("DELETE Lane: Failed to load graph file [" + file_path + "]");
    }

    //judge if the floor is exist
    bool floor_exist_flag = false;
    YAML::Node level_node = graph_config["levels"];
    for(auto iter=level_node.begin(); iter!=level_node.end(); iter++)
    {
      if(iter->first.as<std::string>() == floor_name)
        floor_exist_flag=true;
    }
    if(floor_exist_flag == false)
    {
      ROS_ERROR("DELETE Lane: floor not exist,abort the request");
      return false;
    }

    //start search for the node
    bool lane_exist_flag = false;
    YAML::Node lanes = level_node[floor_name]["lanes"];
    for(auto iter = lanes.begin(); iter != lanes.end(); iter++)
    {
      YAML::Node lane = *iter;
      if(lane[0].as<std::size_t>() == start_wp && lane[1].as<std::size_t>() == goal_wp)
      {
        lanes.remove(lane);
        lane_exist_flag = true;
        ROS_INFO("DELETE Lane: find lane,delete success");
      }
    }
    if(lane_exist_flag == false)
    {
      ROS_ERROR("DELETE Lane: not find the lane,abort the request");
      return false;
    }

    //clear the file will be written
    std::ofstream file_writer(file_path, std::ios_base::out);
    //std::cout<< graph_config<<std::endl;
    // save the file
    file<<graph_config<<std::endl;

    //close the file
    file_writer.close();
    file.close();
    return true;

  }

  std::string _nav_file_path;
  std::string _config_path;
  //to record the current nav file waypoint num and lane num,to give back a index
  std::size_t _waypoint_num;
  std::size_t _lane_num;
};

agx_scheduler_node::agx_scheduler_node()
{
  //init ros
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  //init ros parameters
  private_nh.param<std::string>("nav_file_path",_graph_file_path,"/");
  private_nh.param<std::string>("config_path",_config_path,"/");
  
  schedule_path_pub = nh.advertise<agx_scheduler::SchedulePath>("/schedule_path", 10, true);
  add_waypoint_server = nh.advertiseService("/agx_scheduler_node/add_waypoint_srv",
                                &agx_scheduler_node::add_waypoint_callback,this);
  add_lane_server = nh.advertiseService("/agx_scheduler_node/add_lane_srv",
                                &agx_scheduler_node::add_lane_callback,this);
  delete_lane_server = nh.advertiseService("/agx_scheduler_node/delete_lane_srv",
                                &agx_scheduler_node::delete_lane_callback,this);
  add_waypoint_client = nh.serviceClient<agx_scheduler::add_waypoint>("/agx_scheduler_node/add_waypoint_srv");

  //init graph method
  _graph_impl_ptr = std::make_shared<GraphImplementation>(_graph_file_path,_config_path);

  //init graph file path
  _graph  = _graph_impl_ptr->make_graph();

  //init search method
  init_search_method();
}

bool agx_scheduler_node::init_search_method()
{
  //init greedy search method
  _greedy_impl_ptr = std::make_shared<GreedyImplementation>(_graph->num_waypoints());

  //init astart search method
  _astar_impl_ptr = std::make_shared<AStarImplementation>(_graph->num_waypoints());

  return true;
}


class agx_scheduler_node::GreedyImplementation
{
  public:    
    GreedyImplementation(std::size_t&& waypoint_num):_waypoint_num(waypoint_num)
    {
      ROS_INFO("Greedy method init,waypoint_num:%ld",_waypoint_num);

      //init list
      _closed_list.reserve(_waypoint_num);
      _parent_list.reserve(_waypoint_num);

      while(_closed_list.size() < _waypoint_num)
        _closed_list.push_back(NULL_INDEX);
      while(_parent_list.size() < _waypoint_num)
        _parent_list.push_back(NULL_INDEX);
      
    }

    //search node for greedysearch method
    struct Node
    {
      std::size_t index;
      double range_to_goal;
      std::size_t parent_index;  
    };

    double get_distance_to_goal(const Eigen::Vector2d& start,const Eigen::Vector2d& goal)
    {
      return (goal-start).norm();
    }

    bool judge_if_seach_success()
    {
      return search_success;
    }

    void set_search_complete()
    {
      search_success=true;
    }
    template<typename waypoint>
    struct GreedyCompare
        {

          GreedyCompare(){}

          bool operator()(const waypoint& wp_a, const waypoint& wp_b)
            {
               return wp_b.range_to_goal < wp_a.range_to_goal;
            }
        };

    using GreedySearchQueue =
        std::priority_queue<Node,std::vector<Node>,
        GreedyCompare<Node>>;

    GreedySearchQueue _search_queue;
    std::vector<std::size_t> _closed_list;
    std::vector<std::size_t> _parent_list;

    std::size_t _waypoint_num;
    bool search_success = false;
};

class agx_scheduler_node::AStarImplementation
{
public:
  AStarImplementation(std::size_t&& waypoint_num):_waypoint_num(waypoint_num)
  {
    ROS_INFO("AStart method init,waypoint_num:%ld",_waypoint_num);

    //init list
    _closed_list.reserve(_waypoint_num);
    _parent_list.reserve(_waypoint_num);

    while(_closed_list.size() < _waypoint_num)
      _closed_list.push_back(NULL_INDEX);
    while(_parent_list.size() < _waypoint_num)
      _parent_list.push_back(NULL_INDEX);
  }

  struct Node
  {
    std::size_t index;
    //Eigen::Vector2d location;
    double heuristic_value;
    double move_cost;
    double total_cost;
  };

  double get_distance(const Eigen::Vector2d& start,const Eigen::Vector2d& goal)
  {
    return (goal-start).norm();
  }

  // double caculate_heuristic_value(const Node& node,const Eigen::Vector2d& start,const Eigen::Vector2d& goal)
  // {
  //   node.heuristic_value = (goal-start).norm();
  // }

  void caculate_total_cost(Node& node)
  {
    node.total_cost = node.heuristic_value + node.move_cost;
  }

  bool judge_if_seach_success()
  {
    return search_success;
  }

  void set_search_complete()
  {
    search_success=true;
  }

  template<typename waypoint>
  struct AStarCompare
      {
        AStarCompare(){}

        bool operator()(const waypoint& wp_a, const waypoint& wp_b)
          {
             return wp_b.total_cost < wp_a.total_cost;
          }
      };

  using AStarSearchQueue =
      std::priority_queue<Node,std::vector<Node>,
      AStarCompare<Node>>;

  AStarSearchQueue _search_queue;
  
  std::vector<std::size_t> _closed_list;
  std::vector<std::size_t> _parent_list;

  std::size_t _waypoint_num;
  bool search_success = false;

};


bool agx_scheduler_node::greedy_search_start()
{
  ros::Time begin=ros::Time::now(); 
  //init start
  GreedyImplementation::Node start_node,goal_node;

  start_node = {_start.index,_greedy_impl_ptr->get_distance_to_goal(_start.location,_goal.location),NULL_INDEX};
  goal_node = {_goal.index,0,NULL_INDEX};
  _greedy_impl_ptr->_search_queue.push(start_node);

  ROS_INFO("search_queue init,queue size: %ld,top index:%ld",
    _greedy_impl_ptr->_search_queue.size(),_greedy_impl_ptr->_search_queue.top().index);
  
  //start search
  while(!(_greedy_impl_ptr->judge_if_seach_success()))
  {
    const GreedyImplementation::Node& top = _greedy_impl_ptr->_search_queue.top();
    const std::size_t top_index = top.index;
    const std::vector<std::size_t>& lanes = _graph->lanes_from(top.index);
    ROS_DEBUG("GET vector lanes with size: %ld",lanes.size());

    _greedy_impl_ptr->_closed_list[top_index] = 1;
    _greedy_impl_ptr->_search_queue.pop();
    ROS_DEBUG("POP Node index: %ld",top_index);

    for(auto iter=lanes.begin(); iter!= lanes.end(); iter++)
    {
      const auto index = _graph->get_lane(*iter).exit().waypoint_index();
      if(_greedy_impl_ptr->_closed_list[index] == 1)
        continue;

      GreedyImplementation::Node node_input =
      {
        index,
        _greedy_impl_ptr->get_distance_to_goal(_graph->get_waypoint(index).get_location(), _goal.location),
        top_index
      };
      _greedy_impl_ptr->_search_queue.push(node_input);
      _greedy_impl_ptr->_parent_list[index] = top_index;

      ROS_DEBUG("PUSH node index:%ld,distance: %f,parent: %ld",index,node_input.range_to_goal,node_input.parent_index);
      if(index == _goal.index)
      {
        //set complete to break the while
        _greedy_impl_ptr->set_search_complete();
        break;
      }
    }
  }

  //start to reconstruct the path
  ROS_INFO("\033[32m START PUB the path, goal index: \033[0m");
  std::size_t path_index = _greedy_impl_ptr->_parent_list[_goal.index];

  agx_scheduler::SchedulePath path_msg;
  agx_scheduler::Waypoint wp_msg;
  wp_msg.index = _goal.index;
  path_msg.path.push_back(wp_msg);

  ROS_INFO("----goal %ld",_goal.index);
  while(path_index != _start.index)
  {
    wp_msg.index = path_index;
    path_msg.path.push_back(wp_msg);

    ROS_INFO(("-------- %ld"),path_index);
    path_index = _greedy_impl_ptr->_parent_list[path_index];
  }
  wp_msg.index = _start.index;
  path_msg.path.push_back(wp_msg);

  //pub the path msg
  schedule_path_pub.publish(path_msg);
  //caculate running time
  ros::Duration running_tim = ros::Time::now() - begin;

  ROS_INFO("---start %ld",_start.index);
  ROS_INFO("GREEDY search complete! Spend time(ms): %f",(float)(running_tim.nsec)/1000000.0);
  return true;
}

bool agx_scheduler_node::astar_search_start()
{
  //init search queue
  ros::Time begin=ros::Time::now(); 
  AStarImplementation::Node start_node;

  start_node = 
  {
    _start.index,
    _astar_impl_ptr->get_distance(_start.location,_goal.location),
    0.0,
    _astar_impl_ptr->get_distance(_start.location,_goal.location),
  };

  _astar_impl_ptr->_search_queue.push(start_node);

  ROS_INFO("search_queue init,queue size: %ld,top index:%ld",
    _astar_impl_ptr->_search_queue.size(),_astar_impl_ptr->_search_queue.top().index);


  //start astar search
  while(!(_astar_impl_ptr->judge_if_seach_success()))
  {
    const AStarImplementation::Node& top = _astar_impl_ptr->_search_queue.top();
    const std::size_t top_index = top.index;
    const Eigen::Vector2d top_location = _graph->get_waypoint(top_index).get_location();
    const double current_move_cost = top.move_cost;

    const std::vector<std::size_t>& lanes = _graph->lanes_from(top.index);
    ROS_DEBUG("GET vector lanes with size: %ld",lanes.size());

    _astar_impl_ptr->_closed_list[top_index] = 1;
    _astar_impl_ptr->_search_queue.pop();
    ROS_DEBUG("POP Node index: %ld",top_index);

    for(auto iter=lanes.begin(); iter!= lanes.end(); iter++)
    {
      const auto index = _graph->get_lane(*iter).exit().waypoint_index();
      const Eigen::Vector2d location_input = _graph->get_waypoint(index).get_location();

      if(_astar_impl_ptr->_closed_list[index] == 1)
        continue;

      AStarImplementation::Node node_input =
      {
        index,
        _astar_impl_ptr->get_distance(location_input, _goal.location),
        current_move_cost + _astar_impl_ptr->get_distance(top_location, location_input),
        0
      };
      _astar_impl_ptr->caculate_total_cost(node_input);

      _astar_impl_ptr->_search_queue.push(node_input);
      _astar_impl_ptr->_parent_list[index] = top_index;

      ROS_DEBUG("PUSH node index:%ld",index);
      if(index == _goal.index)
      {
        //set complete to break the while
        _astar_impl_ptr->set_search_complete();
        break;
      }
    }
  }

  //start to reconstruct the path
  ROS_INFO("\033[32m START PUB the path, goal index: \033[0m");
  std::size_t path_index = _astar_impl_ptr->_parent_list[_goal.index];

  agx_scheduler::SchedulePath path_msg;
  agx_scheduler::Waypoint wp_msg;
  wp_msg.index = _goal.index;
  path_msg.path.push_back(wp_msg);

  ROS_INFO("----goal %ld",_goal.index);
  while(path_index != _start.index)
  {
    wp_msg.index = path_index;
    path_msg.path.push_back(wp_msg);

    ROS_INFO(("-------- %ld"),path_index);
    path_index = _astar_impl_ptr->_parent_list[path_index];
  }
  wp_msg.index = _start.index;
  path_msg.path.push_back(wp_msg);

  //pub the path msg
  schedule_path_pub.publish(path_msg);
  //caculate running time
  ros::Duration running_tim = ros::Time::now() - begin;

  ROS_INFO("---start %ld",_start.index);
  ROS_INFO("AStar search complete! Spend time(ms): %f",(float)(running_tim.nsec)/1000000.0);
  return true;

}

bool agx_scheduler_node::add_waypoint_to_graph(Eigen::Vector2d location, std::string name, 
                                std::string floor_name,std::string nav_file_name)
{
  return _graph_impl_ptr->add_waypoint_to_graph(location, name, 
                                                floor_name,nav_file_name);
}

bool agx_scheduler_node::add_lanes_to_graph(std::size_t start_wp, std::size_t goal_wp,
                        std::string floor_name,std::string nav_file_name)
{
  //
  return _graph_impl_ptr->add_lanes_to_graph(start_wp, goal_wp,
                                             floor_name, nav_file_name);
}

bool agx_scheduler_node::delete_lane_of_graph(std::size_t start_wp, std::size_t goal_wp,
                               std::string floor_name, std::string nav_file_name)
{
  return _graph_impl_ptr->delete_lane_of_graph(start_wp,goal_wp,floor_name,nav_file_name);
}


bool agx_scheduler_node::set_goal_and_start(void)
{
  std::size_t start_index,goal_index;
  std::cout<<"please write the start waypoint index"<<std::endl;
  std::cin >> start_index;
  std::cout<<"please write the goal waypoint index"<<std::endl;
  std::cin >> goal_index;

  _start.index = start_index;
  _start.location = _graph->get_waypoint(start_index).get_location();

  _goal.index = goal_index;
  _goal.location = _graph->get_waypoint(goal_index).get_location();

  ROS_INFO("goal set success,start index: %ld ,goal index: %ld ",get_start_index(),get_goal_index());

  return true;
}

bool agx_scheduler_node::create_nav_yaml()
{
  //
  return true;
}

std::size_t agx_scheduler_node::get_goal_index()
{
  return _goal.index;
}

std::size_t agx_scheduler_node::get_start_index()
{
  return _start.index;
}

bool agx_scheduler_node::add_waypoint_callback(agx_scheduler::add_waypoint::Request& request, 
                                               agx_scheduler::add_waypoint::Response& response)
{
  ROS_INFO("\033[32mSERVICE CALL: add_waypoint\033[0m");
  ros::Time begin=ros::Time::now(); 

  bool return_flag = add_waypoint_to_graph({request.location_x,request.location_y}, request.waypoint_name, request.floor_name,request.file_name);
  //get waypoint index,the index = waypoint_num -1
  const std::size_t waypoint_index = (_graph_impl_ptr->_waypoint_num) - 1;
  if(return_flag)
  {
    response.success = true;
    response.index = waypoint_index;
    //caculate running time
    ros::Duration running_tim = ros::Time::now() - begin;

    ROS_INFO("\033[32mSERVICE CALL: add_waypoint index: %ld ,use time(ms): %f\033[0m",waypoint_index,(float)(running_tim.nsec)/1000000.0);
  }
  else
  {
    response.success = false;
    ROS_ERROR("SERVICE CALL: add waypoint failed!");
  }
  return true;
}

bool agx_scheduler_node::add_lane_callback(agx_scheduler::add_lane::Request& request, 
                                           agx_scheduler::add_lane::Response& response)
{
  ROS_INFO("\033[32mSERVICE CALL: add_lane\033[0m");
  ros::Time begin=ros::Time::now(); 

  bool return_flag = add_lanes_to_graph(request.start_index,request.end_index,request.floor_name,request.file_name);
  if(request.double_side == true)
    bool return_flag_2 = add_lanes_to_graph(request.end_index,request.start_index,request.floor_name,request.file_name);
  
  //get lane index,the same with waypoint index
  const std::size_t lane_index = (_graph_impl_ptr->_lane_num) - 1;
  if(return_flag)
  {
    response.success = true;
    response.index = lane_index;
    //caculate running time
    ros::Duration running_tim = ros::Time::now() - begin;
    ROS_INFO("\033[32mSERVICE CALL: add_lane index: %ld ,use time(ms): %f\033[0m",lane_index,(float)(running_tim.nsec)/1000000.0);
  }
  else
  {
    response.success = false;
    ROS_ERROR("SERVICE CALL: add lane failed!");
  }
  return true;
}

bool agx_scheduler_node::delete_lane_callback(agx_scheduler::delete_lane::Request& request,
                                              agx_scheduler::delete_lane::Response& response)
{
  ROS_INFO("\033[32mSERVICE CALL: delete_lane\033[0m");
  ros::Time begin=ros::Time::now(); 
  bool return_flag = delete_lane_of_graph(request.start_index,request.end_index,request.floor_name,request.file_name);
  if(return_flag)
  {
    response.success = true;
    //caculate running time
    ros::Duration running_tim = ros::Time::now() - begin;
    ROS_INFO("\033[32mSERVICE CALL: delete_lane,use time(ms): %f\033[0m",(float)(running_tim.nsec)/1000000.0);
  }
  else
  {
    response.success = false;
    ROS_ERROR("SERVICE CALL: delete lane failed!");
  }
  return true;
}
// }//namespace AgileX


int main(int argc, char * argv[])
{
  ROS_INFO("----------------------------");
  ros::init(argc, argv, "limo_node");
  ros::NodeHandle nh;

  agx_scheduler_node agx_node;

  //agx_node.set_goal_and_start(); 

  //agx_node.greedy_search_start();

  //agx_node.astar_search_start();

  // agx_node.add_waypoint_to_graph({1.56,2.78}, "","L1","test");
  // agx_node.add_waypoint_to_graph({3.56,8.78}, "","L1","test");
  // agx_node.add_lanes_to_graph(7,8,"L1","test.yaml");
  ros::spin();
  return 0;
}