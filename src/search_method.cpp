#include "search_method.hpp"


double search_solver::caculate_euler_cost(Eigen::Vector2d positon_current,Eigen::Vector2d position_goal)
{
    //
    return (position_goal-positon_current).norm();
}

