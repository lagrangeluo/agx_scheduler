#include "agx_scheduler/agx_scheduler_node.hpp"

// namespace AgileX{


using namespace AgileX;

agx_scheduler_node::agx_scheduler_node()
{
    //init ros
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("nav_file_path",_graph_file_path,"/"); 

    //init graph file path
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

  // _graph_file_path = "/home/agilex/schedule_ws/src/agx_scheduler/config/nav.yaml";

  _graph  = std::make_shared<rmf_traffic::agv::Graph>(agx_scheduler::parse_graph(_graph_file_path, traits));
  
  //init graph
  ROS_INFO("graph parse complete,waypoint num: %ld, lane num: %ld",_graph->num_waypoints(),_graph->num_lanes());
  if(_graph->num_waypoints()<1)
    ROS_ERROR("waypoint num is zero!");

  //init greedy search method
  _greedy_impl_ptr = std::make_shared<GreedyImplementation>(_graph->num_waypoints());
  // const Eigen::Vector2d info = _graph->get_waypoint(std::size_t(0)).get_location();
  // std::cout<<"x:"<<info[0]<<"y: "<<info[1]<<std::endl;
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
    ROS_INFO("GET vector lanes with size: %ld",lanes.size());

    _greedy_impl_ptr->_closed_list[top_index] = 1;
    _greedy_impl_ptr->_search_queue.pop();
    ROS_INFO("POP Node index: %ld",top_index);

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

      ROS_INFO("PUSH node index:%ld,distance: %f,parent: %ld",index,node_input.range_to_goal,node_input.parent_index);
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
  ROS_INFO("----goal %ld",_goal.index);
  while(path_index != _start.index)
  {
    ROS_INFO(("-------- %ld"),path_index);
    path_index = _greedy_impl_ptr->_parent_list[path_index];
  }

  ros::Duration running_tim = ros::Time::now() - begin;

  ROS_INFO("---start %ld",_start.index);
  ROS_INFO("GREEDY search complete! Spend time(ms): %f",(float)(running_tim.nsec)/1000000.0);
  return true;
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

std::size_t agx_scheduler_node::get_goal_index()
{
  return _goal.index;
}

std::size_t agx_scheduler_node::get_start_index()
{
  return _start.index;
}

// }//namespace AgileX


int main(int argc, char * argv[])
{
  ROS_INFO("----------------------------");
  ros::init(argc, argv, "limo_node");
  ros::NodeHandle nh;

  agx_scheduler_node agx_node;

  agx_node.set_goal_and_start(); 

  agx_node.greedy_search_start();

  ros::spin();
  return 0;
}