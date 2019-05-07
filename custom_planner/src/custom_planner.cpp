#include <pluginlib/class_list_macros.h>
#include "custom_planner.h"

#include "yaml-cpp/yaml.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(custom_planner::CustomPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace custom_planner {

CustomPlanner::CustomPlanner (){
    ROS_INFO("Using plugin \"custom_planner\"");
}

CustomPlanner::CustomPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}


void CustomPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    _initialized = false;
    initialize();
}

void CustomPlanner::initialize(){
    if(!_initialized){
        

    }
}

bool CustomPlanner::makePlan(const geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    geometry_msgs::PoseStamped a, b, c, d;

    a.header.frame_id = "map";
    b.header.frame_id = "map";
    c.header.frame_id = "map";
    d.header.frame_id = "map";

    tf::Quaternion qa = tf::createQuaternionFromYaw(0.0);
    tf::Quaternion qb = tf::createQuaternionFromYaw(1.57);
    tf::Quaternion qc = tf::createQuaternionFromYaw(3.14);
    tf::Quaternion qd = tf::createQuaternionFromYaw(-1.57);

    a.pose.position.x = 1.0;
    a.pose.position.y = 0.0;

    a.pose.orientation.x = qa.x();
    a.pose.orientation.y = qa.y();
    a.pose.orientation.z = qa.z();
    a.pose.orientation.w = qa.w();

    b.pose.position.x = 1.0;
    b.pose.position.y = 1.0;

    b.pose.orientation.x = qb.x();
    b.pose.orientation.y = qb.y();
    b.pose.orientation.z = qb.z();
    b.pose.orientation.w = qb.w();

    c.pose.position.x = 0.0;
    c.pose.position.y = 1.0;

    c.pose.orientation.x = qc.x();
    c.pose.orientation.y = qc.y();
    c.pose.orientation.z = qc.z();
    c.pose.orientation.w = qc.w();

    d.pose.position.x = 0.0;
    d.pose.position.y = 0.0;

    d.pose.orientation.x = qd.x();
    d.pose.orientation.y = qd.y();
    d.pose.orientation.z = qd.z();
    d.pose.orientation.w = qd.w();

    plan.push_back(start);
    plan.push_back(a);
    plan.push_back(b);
    plan.push_back(c);
    plan.push_back(d);
    plan.push_back(goal);

    return true;
}

bool CustomPlanner::setPath(const std::vector<geometry_msgs::PoseStamped>& path){
    _custom_path = path;
    return true;
}

};