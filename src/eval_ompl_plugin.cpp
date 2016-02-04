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
* Author: Christian Connette
*********************************************************************/

#include <ros/ros.h>

// ros sandbox classes
#include <ompl_planner_base/OMPLPlannerBaseStats.h>
#include <ompl_ros_interface/OmplPlannerDiagnostics.h>

// std c++ classes
#include <math.h>
#include <string.h>


/**
 * @class NodeClass
 * @brief This implements a simple Node for evaluation of the ompl_planner_plugin. The node listens to the statistics_ompl and diagnostics_ompl topics and calcs some refernce values (mean, root-mean-square (rms-)deviation, ...)
 */
class NodeClass
{
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n_handle_;

		// topics to subscribe
		ros::Subscriber topic_sub_diagnostics_; // receives diagnostics considering the ompl planner
		ros::Subscriber topic_sub_statistics_; // some values additional to diagnostics considering the plugin

		// member variables

		// general
		int num_plan_success_, num_plan_request_; // number of plan querries and successfeul planning attempts
		std::string planner_name_;

		// diagnostics topic
		std::vector<double> planning_time_; // time ompl needed to find a path
		std::vector<double> trajectory_size_; // number of frames in the trajectory (0 if no path found)
		std::vector<double> state_allocator_size_; // number of seeds generated for the state during planning

		// statistics topic
		std::vector<double> start_goal_dist_; // direct distance between start and goal pose
		std::vector<double> path_length_; // length of path found from start to goal pose
		std::vector<double> total_planning_time_; // total time between assigning a start and goal pose to the planner and output of final path

		// characteristic values for statistics (diagnostics topic)
		double max_salloc_size_, min_salloc_size_, max_traj_size_, min_traj_size_, max_plan_time_, min_plan_time_;
		double sum_salloc_size_, sum_traj_size_, sum_plan_time_; // sum over all values (for calculation of mean)
		double sqsum_salloc_size_, sqsum_traj_size_, sqsum_plan_time_; // sum of squared values (for calculation of root-mean-square deviation)

		// characteristic values for statistics (statistics topic)
		double max_startgoal_dist_, min_startgoal_dist_, max_path_length_, min_path_length_, max_plan_tottime_, min_plan_tottime_;
		double sum_startgoal_dist_, sum_path_length_, sum_plan_tottime_; // sum over all values (for calculation of mean)
		double sqsum_startgoal_dist_, sqsum_path_length_, sqsum_plan_tottime_; // sum of squared values (for calculation of root-mean-square deviation)


		// Constructor
		NodeClass()
		{
			// initialize vectors --> clear
			planning_time_.clear();
			trajectory_size_.clear();
			state_allocator_size_.clear();
			start_goal_dist_.clear();
			path_length_.clear();
			total_planning_time_.clear();

			// assign topics
			topic_sub_diagnostics_ = n_handle_.subscribe("/move_base_node/OMPLPlannerBase/diagnostics_ompl", 1, &NodeClass::topicCallbackDiagnostics, this);
			topic_sub_statistics_ = n_handle_.subscribe("/move_base_node/OMPLPlannerBase/statistics_ompl", 1, &NodeClass::topicCallbackStatistics, this);

			// init counters
			num_plan_success_ = 0;
			num_plan_request_ = 0;

			// init characteristic values for statistics (diagnostics_ompl)
			max_salloc_size_ = 0.0;
			min_salloc_size_ = 1000000;
			max_traj_size_ = 0.0;
			min_traj_size_ = 1000000;
			max_plan_time_ = 0.0;
			min_plan_time_ = 1000000.0;
			// sum values
			sum_salloc_size_ = 0.0;
			sum_traj_size_ = 0.0;
			sum_plan_time_ = 0.0;
			// sum of squared values (for calculation of root-mean-square deviation)
			sqsum_salloc_size_ = 0.0;
			sqsum_traj_size_ = 0.0;
			sqsum_plan_time_ = 0.0;

			// init characteristic values for statistics (statistics_ompl)
			max_startgoal_dist_ = 0.0;
			min_startgoal_dist_ = 1000000.0;
			max_path_length_ = 0.0;
			min_path_length_ = 1000000.0;
			max_plan_tottime_ = 0.0;
			min_plan_tottime_ = 1000000.0;
			// sum values
			sum_startgoal_dist_ = 0.0;
			sum_path_length_ = 0.0;
			sum_plan_tottime_ = 0.0;
			// sum of squared values (for calculation of root-mean-square deviation)
			sqsum_startgoal_dist_ = 0.0;
			sqsum_path_length_ = 0.0;
			sqsum_plan_tottime_ = 0.0;
		}

		// Destructor
		~NodeClass(){}


		// topic Callbacks

		/**
		 * @brief  Callback for Diagnostics topic. This calculates some statistical measures for the diagnostic data. Every time a message is received the characteristic values are updated
		 * @param  Pointer to the received diagnostics message
		 */
		void topicCallbackDiagnostics(const ompl_ros_interface::OmplPlannerDiagnostics::ConstPtr& msg)
		{
			// instantiate temporary variables
			double salloc_size, traj_size, plan_time;

			// increment counter for planning attempts
			num_plan_request_ = num_plan_request_ + 1;

			// check whether planning was successful (otherwise suppress input)
			if(msg->trajectory_size > 0)
			{
				// successfull -> increment counter and get the values of interest from the msg
				num_plan_success_ = num_plan_success_ + 1;
				salloc_size = (double) msg->state_allocator_size;
				traj_size = (double) msg->trajectory_size;
				plan_time = (double) msg->planning_time;

				// ... and write it to the vectors
				state_allocator_size_.push_back(salloc_size);
				trajectory_size_.push_back(traj_size);
				planning_time_.push_back(plan_time);

				// calculate characteristic values for statistics

				// get min/max values
				if(salloc_size > max_salloc_size_)
					max_salloc_size_ = salloc_size;
				if(salloc_size < min_salloc_size_)
					min_salloc_size_ = salloc_size;

				if(traj_size > max_traj_size_)
					max_traj_size_ = traj_size;
				if(traj_size < min_traj_size_)
					min_traj_size_ = traj_size;

				if(plan_time > max_plan_time_)
					max_plan_time_ = plan_time;
				if(plan_time < min_plan_time_)
					min_plan_time_ = plan_time;

				// sum up values values
				sum_salloc_size_ = sum_salloc_size_ + salloc_size;
				sum_traj_size_ = sum_traj_size_ + traj_size;
				sum_plan_time_ = sum_plan_time_ + plan_time;

				// sum of squared values (for calculation of root-mean-square deviation)
				sqsum_salloc_size_ = sqsum_salloc_size_ + (salloc_size*salloc_size);
				sqsum_traj_size_ = sqsum_traj_size_ + (traj_size*traj_size);
				sqsum_plan_time_ = sqsum_plan_time_ + (plan_time*plan_time);
			}

			// as soon as there are more than 2 plans made (huge number of samples;) --> print statistics
			if(num_plan_success_ > 2)
			{
				// general
				double success_rate = ((double) num_plan_success_)/((double) num_plan_request_);
				ROS_INFO("Plans requested: %d; Planning succeded: %d; Success-Rate: %f", num_plan_request_, num_plan_success_, success_rate);

				// statistics on planning time
				double mean_plan_time = sum_plan_time_/num_plan_success_;
				double sqmean_plan_time = sqsum_plan_time_/num_plan_success_;
				double rmsdev_plan_time = sqrt(sqmean_plan_time - mean_plan_time*mean_plan_time);
				ROS_INFO("Planning time (ompl) [s]: min: %f, max: %f, mean: %f, rms-dev: %f", min_plan_time_, max_plan_time_, mean_plan_time, rmsdev_plan_time);

				// statistics on trajectory size
				double mean_traj_size = sum_traj_size_/num_plan_success_;
				double sqmean_traj_size = sqsum_traj_size_/num_plan_success_;
				double rmsdev_traj_size = sqrt(sqmean_traj_size - mean_traj_size*mean_traj_size);
				ROS_INFO("Trajectory size/statenum (ompl): min: %f, max: %f, mean: %f, rms-dev: %f", min_traj_size_, max_traj_size_, mean_traj_size, rmsdev_traj_size);

				// statistics on state allocator size
				double mean_salloc_size = sum_salloc_size_/num_plan_success_;
				double sqmean_salloc_size = sqsum_salloc_size_/num_plan_success_;
				double rmsdev_salloc_size = sqrt(sqmean_salloc_size - mean_salloc_size*mean_salloc_size);
				ROS_INFO("State allocator size (ompl): min: %f, max: %f, mean: %f, rms-dev: %f", min_salloc_size_, max_salloc_size_, mean_salloc_size, rmsdev_salloc_size);
			}
		}


		
		/**
		 * @brief  Callback for Statistics topic. This calculates some statistical measures for the statistic data. Every time a message is received the characteristic values are updated
		 * @param  Pointer to the received statistic message
		 */
		void topicCallbackStatistics(const ompl_planner_base::OMPLPlannerBaseStats::ConstPtr& msg)
		{
			// instantiate temporary variables
			double startgoal_dist, path_length, plan_tottime;

			// get values from msg and write it to the vectors
			startgoal_dist = msg->start_goal_dist;
			path_length = msg->path_length;
			plan_tottime = msg->total_planning_time;

			// ... and write it to the vectors
			start_goal_dist_.push_back(startgoal_dist);
			path_length_.push_back(path_length);
			total_planning_time_.push_back(plan_tottime);

			// calculate characteristic values for statistics

			// get min/max values
			if(startgoal_dist > max_startgoal_dist_)
				max_startgoal_dist_ = startgoal_dist;
			if(startgoal_dist < min_startgoal_dist_)
				min_startgoal_dist_ = startgoal_dist;

			if(path_length > max_path_length_)
				max_path_length_ = path_length;
			if(path_length < min_path_length_)
				min_path_length_ = path_length;

			if(plan_tottime > max_plan_tottime_)
				max_plan_tottime_ = plan_tottime;
			if(plan_tottime < min_plan_tottime_)
				min_plan_tottime_ = plan_tottime;

			// sum up values
			sum_startgoal_dist_ = sum_startgoal_dist_ + startgoal_dist;
			sum_path_length_ = sum_path_length_ + path_length;
			sum_plan_tottime_ = sum_plan_tottime_ + plan_tottime;

			// sum of squared values (for calculation of root-mean-square deviation)
			sqsum_startgoal_dist_ = sqsum_startgoal_dist_ + (startgoal_dist*startgoal_dist);
			sqsum_path_length_ = sqsum_path_length_ + (path_length*path_length);
			sqsum_plan_tottime_ = sqsum_plan_tottime_ + (plan_tottime*plan_tottime);

			// as soon as there are more than 2 plans made (huge number of samples;) --> print statistics
			if(num_plan_success_ > 2)
			{
				// doublecheck whether data is consistent -> we should receive a statistics_ompl msg for every successful planning attempt
				if(!num_plan_success_ == start_goal_dist_.size())
					ROS_WARN("Consistency check failed - Number of statistics and diagnostics msgs seem not to match");

				// anyway calculate the characteristic values and log them out

				// statistics on path length
				double mean_path_length = sum_path_length_/num_plan_success_;
				double sqmean_path_length = sqsum_path_length_/num_plan_success_;
				double rmsdev_path_length = sqrt(sqmean_path_length - mean_path_length*mean_path_length);
				ROS_INFO("Length of resulting path [m]: min: %f, max: %f, mean: %f, rms-dev: %f", min_path_length_, max_path_length_, mean_path_length, rmsdev_path_length);

				// statistics on distance between start and goal -> to compare with path length
				double mean_startgoal_dist = sum_startgoal_dist_/num_plan_success_;
				double sqmean_startgoal_dist = sqsum_startgoal_dist_/num_plan_success_;
				double rmsdev_startgoal_dist = sqrt(sqmean_startgoal_dist - mean_startgoal_dist*mean_startgoal_dist);
				ROS_INFO("Direct distance from start to goal [m]: min: %f, max: %f, mean: %f, rms-dev: %f", min_startgoal_dist_, max_startgoal_dist_, mean_startgoal_dist, rmsdev_startgoal_dist);

				// statistics on total time for planning
				double mean_plan_tottime = sum_plan_tottime_/num_plan_success_;
				double sqmean_plan_tottime = sqsum_plan_tottime_/num_plan_success_;
				double rmsdev_plan_tottime = sqrt(sqmean_plan_tottime - mean_plan_tottime*mean_plan_tottime);
				ROS_INFO("Total time until output of plan [s]: min: %f, max: %f, mean: %f, rms-dev: %f", min_plan_tottime_, max_plan_tottime_, mean_plan_tottime, rmsdev_plan_tottime);
			}
		}
};



/**
 * @brief  This does actually run the node
 */
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "eval_ompl_planner_plugin");
    
	// construct nodeClass
    NodeClass nodeClass;

	// specify looprate of control cycle
 	ros::Rate loop_rate(10); // Hz

	// keep cycling and listen for topics
	while(nodeClass.n_handle_.ok())
	{
		// process Callbacks
        ros::spinOnce();

		// -> let node sleep for the rest of the cycle
        loop_rate.sleep();
	}

	return 0;
}
















