/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

#include <std_msgs/Bool.h>
#include <thread>
#include <mutex>
#include <unistd.h>

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };
  static const char *movebase_str[] =
          { "PLANNING", "CONTROLLING", "CLEARING" };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  static const char *recovery_trigger_str[] =
          { "PLANNING_R", "CONTROLLING_R", "OSCILLATION_R" };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Publishes a velocity command of turning to the base
       */
      void publishRotateVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      bool isValidPlan( std::vector<geometry_msgs::PoseStamped>& plan );

      tf2_ros::Buffer& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_, num_replans_to_the_samegoal, clearing_retries_ ;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recompute_paths_to_frontiers_pub_, unreachable_frontier_pub_;
      ros::Subscriber goal_sub_, done_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      bool isdone;

      int  max_plan_length_ ;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;
      boost::condition_variable_any planner_cond_;
      geometry_msgs::PoseStamped planner_goal_, previous_goal_; // prev goal added by hkm
      std::vector<geometry_msgs::PoseStamped> m_unreachable_goals; // by hkm
      std::mutex mutex_unreachable_goals; // by hkm

      boost::thread* planner_thread_;

      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;

      bool mb_is_goal_covered ;
    public:

      inline bool planner_goal_equals_to_prevgoal( )
      {
    	  ROS_WARN("prev/curr goal (%f %f), (%f %f) \n",
    			  previous_goal_.pose.position.x, previous_goal_.pose.position.y,
				  planner_goal_.pose.position.x, planner_goal_.pose.position.y
    	  );

    	  float fxdiff = (previous_goal_.pose.position.x - planner_goal_.pose.position.x) ;
    	  float fydiff = (previous_goal_.pose.position.y - planner_goal_.pose.position.y) ;
    	  return std::sqrt( fxdiff * fxdiff + fydiff * fydiff ) < 0.001 ? true : false;
      }

      inline bool is_goal_covered( const geometry_msgs::PoseStamped& goal)
      {
    	  costmap_2d::Costmap2D* pocostmap = planner_costmap_ros_->getCostmap();
    		geometry_msgs::PoseStamped robotpose;
    		getRobotPose(robotpose, planner_costmap_ros_);

    		const double wx_r = robotpose.pose.position.x ;
			const double wy_r = robotpose.pose.position.y ;
    	    const double wx_g = goal.pose.position.x ;
    	    const double wy_g = goal.pose.position.y ;
    	    uint32_t mx_r, my_r, mx_g, my_g;
    	    pocostmap->worldToMap(wx_r, wy_r, mx_r, my_r);
    	    pocostmap->worldToMap(wx_g, wy_g, mx_g, my_g);

    	    float r2gdist_world = std::sqrt( (wx_r - wx_g) * (wx_r - wx_g) + (wy_r - wy_g) * (wy_r - wy_g) ) ;

    		// 0	1	2
    		// 3		5
    		// 6	7	8
    	    //int x0, x1, x2, x3, x5, x6, x7, x8, y0, y1, y2, y3, y5, y6, y7, y8;
    		unsigned int c0 = pocostmap->getCost( mx_r - 1	,	my_r - 1 );
    		unsigned int c1 = pocostmap->getCost( mx_r		,	my_r - 1 );
    		unsigned int c2 = pocostmap->getCost( mx_r + 1	,	my_r - 1 );

    		unsigned int c3 = pocostmap->getCost( mx_r - 1	,  my_r ) ;
    		unsigned int c4 = pocostmap->getCost( mx_r		,  my_r ) ;
    		unsigned int c5 = pocostmap->getCost( mx_r + 1	,  my_r ) ;

    		unsigned int c6 = pocostmap->getCost( mx_r - 1	,  my_r + 1);
    		unsigned int c7 = pocostmap->getCost( mx_r		,  my_r + 1);
    		unsigned int c8 = pocostmap->getCost( mx_r + 1	,  my_r + 1);

//    		ROS_WARN("mx my wx wy ox oy res: %d %d %f %f %f %f \n", mx, my, wx, wy, pocostmap->getOriginX(), pocostmap->getOriginY(), pocostmap->getResolution() );
 //   		ROS_ERROR("\n %u %u %u \n %u %u %u \n %u %u %u \n", c0, c1, c2, c3, c4, c5, c6, c7, c8);

    		if (r2gdist_world < 1.0)
    		{
				if( (c0 == 255 || c1 == 255 || c2 == 255 || c3 == 255 || c4 == 255 ||  c5 == 255 || c6 == 255 || c7 == 255 || c8 == 255 ) )
					return false ;
				else
				{
					ROS_ERROR("This point has been covered online \n");
					true;
				}
    		}
    		else
    			return false ;
      }

      bool is_robot_stuck();


      bool isDone(){ return isdone; };
      void ismappingdoneCB(const std_msgs::BoolPtr& ismappingdone) ;
  };
};
#endif

