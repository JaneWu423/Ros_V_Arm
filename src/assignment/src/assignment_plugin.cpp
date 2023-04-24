#include <ros/ros.h>

#include "assignment_context.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>

#include <class_loader/class_loader.hpp>

#include <dynamic_reconfigure/server.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <std_msgs/String.h>


class CS3891Planner : public planning_interface::PlannerManager{

public:

  CS3891Planner() : planning_interface::PlannerManager() {}

  virtual 
  bool 
  initialize
  ( const robot_model::RobotModelConstPtr& model, 
    const std::string& ns ){

    if (!ns.empty()){
      nh = ros::NodeHandle( ns );
    }
    model_ = model;

    nh.param<std::string>("group_name", group_name, "both_arms");

      
    context.reset( new CS3891Context( model, 
				     std::string( "CS3891" ),
				     group_name,
				     nh ) );
    
    // Subscribe to the MoveGroup topic to receive updates on group name
    group_name_sub = nh.subscribe("/move_group/motion_plan_request", 1, &CS3891Planner::group_name_callback, this);


    return true;

  }

 void group_name_callback(const moveit_msgs::MotionPlanRequest::ConstPtr& msg)
{
    std::string group_name = msg->group_name;
    ROS_INFO("Received group name update: %s", group_name.c_str());

    // Update the planning context with the new group name
    context.reset(new CS3891Context(model_, "CS3891", group_name, nh));
}
  virtual
  bool 
  canServiceRequest
  ( const moveit_msgs::MotionPlanRequest &req ) const 
  { return true; }

  virtual std::string getDescription() const
  { return std::string( "CS3891Planner" ); }

  virtual
  void 
  getPlanningAlgorithms
  ( std::vector<std::string> &algs ) const{
    algs.resize(1);
    algs[0] = "CS3891Planner";
  }

  virtual 
  planning_interface::PlanningContextPtr 
  getPlanningContext
  ( const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes &error_code) const{

    context->setPlanningScene( planning_scene );
    context->setMotionPlanRequest( req );
    
return context;

}

  virtual 
  void 
  setPlannerConfigurations
  (const planning_interface::PlannerConfigurationMap &pconfig){}

private:
  
  ros::NodeHandle nh;
  planning_interface::PlanningContextPtr  context;

  ros::Subscriber group_name_sub;
  std::string group_name;  // Member variable for group name
  robot_model::RobotModelConstPtr model_;

};

CLASS_LOADER_REGISTER_CLASS( CS3891Planner, planning_interface::PlannerManager );
