/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
* Author: Christian Connette, Eitan Marder-Eppstein
*********************************************************************/

#ifndef OMPL_PLANNER_BASE_H
#define OMPL_PLANNER_BASE_H

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_global_planner.h>

// other ros classes
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// ros sandbox classes
#include <ompl_planner_base/OMPLPlannerBaseStats.h>
#include <ompl_ros_interface/OmplPlannerDiagnostics.h>

// std c++ classes
#include <math.h>
#include <string.h>

// boost classes
#include <boost/bind.hpp>

// ompl planner specific classes
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
// #include <ompl/base/StateManifold.h>
// #include <ompl/base/manifolds/SE2StateManifold.h>
// #include <ompl/base/manifolds/RealVectorBounds.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
// ompl planners
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/BasicPRM.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/sbl/SBL.h>


namespace ompl_planner_base{

/**
 * @class OMPLPlannerBase
 * @brief Plugin to the ros base_global_planner. Implements an interface to the Open Motion Planning Library OMPL
 */
class OMPLPlannerBase : public nav_core::BaseGlobalPlanner {

	public:
		/**
		 * @brief  Constructor for the PRM Planner
		 */	
		OMPLPlannerBase();

		/**
		 * @brief  Constructor for the PRM Planner
		 * @param  name The name of this planner
		 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
		 */
		OMPLPlannerBase(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief  Initialization function for the PRM Planner
		 * @param  name The name of this planner
		 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
		 */
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief Given a goal pose in the world, compute a plan
		 * @param start The start pose 
		 * @param goal The goal pose 
		 * @param plan The plan... filled by the planner
		 * @return True if a valid plan was found, false otherwise
		 */
		bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

		/**
		 * @brief  Destructor for the PRM Planner
		 */	
		~OMPLPlannerBase();


	private:

		ros::NodeHandle private_nh_;
		costmap_2d::Costmap2DROS* costmap_ros_;
		double step_size_, min_dist_from_robot_;
		costmap_2d::Costmap2D costmap_;
		base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
		double inscribed_radius_, circumscribed_radius_, inflation_radius_;
		std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot
		bool initialized_;

		// parameters to configure planner
		bool interpolate_path_; ///<@brief parameter to flag whether path shall be interpolated (set to false for Elastic Bands)
		bool publish_diagnostics_; ///<@brief parameter to flag whether diagnostic and statistic msgs shall be published (default false)
		int max_footprint_cost_; ///<@brief maximum cost for which footprint is still treated as collision free
		double relative_validity_check_resolution_; ///<@brief resolution of validity checkeing of robot motion
		double max_dist_between_pathframes_; ///<@brief parameter to set density of pathframes for interpolation
		std::string planner_type_; ///<@brief parameter to switch between different planners provided through ompl

		// Topics & Services
		ros::Publisher plan_pub_; ///<@brief topic used to publish resulting plan for visualization
		ros::Publisher diagnostic_ompl_pub_; ///<@brief topic used to publish some diagnostic data about the results of the ompl
		ros::Publisher stats_ompl_pub_; ///<@brief topic used to publish some statistics about the planner plugin

		/**
		 * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
		 * @param x_i The x position of the robot 
		 * @param y_i The y position of the robot 
		 * @param theta_i The orientation of the robot
		 * @return 
		 */
		double footprintCost(double x_i, double y_i, double theta_i);

		/**
		 * @brief Interface class to footprint check for ompl planning library
		 * @param state_SE2 The pose of the robot in the plaine as provided by ompl SE2 manifold (x, y, yaw)
		 * @return true if pose valid, false otherwise
		 */
		bool isStateValid2DGrid(const ompl::base::State *state);

		/**
		 * @brief Interpolates path returned from ompl to fit density-requirements of local planner
		 * @param Path to be interpolated as vector of Pose2D
		 * @return true if interpolation succesful
		 */
		bool interpolatePathPose2D(std::vector<geometry_msgs::Pose2D>& path);

		// Visualization

		/**
		 * @brief Publish a path for visualization purposes
		 * @param Path to be published as nav Message (nav_msgs::plan)
		 */
		void publishPlan(std::vector<geometry_msgs::PoseStamped> path);

		// Configuration

		/**
		 * @brief Read desired planner type from parameter server and set according ompl planner to simple setup
		 * @param Reference to SimpleSetup
		 */
		void setPlannerType(ompl::geometric::SimpleSetup& simple_setup);

		// Type Conversions

		/**
		 * @brief Converts an OMPL State of Type SE2 to a ROS Pose2D type
		 * @param Reference to OMPL State (here state of type SE2 which shall be converted)
		 * @param Converted ROS Pose2D State
		 */
		void OMPLStateSE2ToROSPose2D(const ompl::base::State* ompl_state, geometry_msgs::Pose2D& pose2D);

		/**
		 * @brief Converts an OMPL ScopedState of Type SE2 to a ROS Pose2D type
		 * @param ScopedState Template Refernce to StateType (here state of type SE2 which shall be converted)
		 * @param Converted ROS Pose2D State
		 */
		void OMPLScopedStateSE2ToROSPose2D(const ompl::base::ScopedState<> scoped_state,
											geometry_msgs::Pose2D& pose2D);

		/**
		 * @brief Converts a ROS Pose2D type to an OMPL ScopedState of Type SE2
		 * @param ScopedState Template Refernce to StateType (here state of type SE2 which has been converted)
		 * @param ROS Pose2D State which shall be converted
		 */
		void ROSPose2DToOMPLScopedStateSE2(ompl::base::ScopedState<>& scoped_state,
											const geometry_msgs::Pose2D pose2D);

		/**
		 * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from quaternions to euler angles)
		 * @param Pose which shall be converted
		 * @param References to converted ROS Pose2D frmae
		 */
		void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D);

		/**
		 * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from euler angles to quaternions, -> z-coordinate is set to zero)
		 * @param References to converted ROS Pose2D frame
		 * @param Pose2D which shall be converted
		 */
		void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D);

	};
};  
#endif
