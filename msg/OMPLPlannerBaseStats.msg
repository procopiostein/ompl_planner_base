# defines some interesting values additional to OMPLPlannerBaseDiagnostics

# Initial conditions of  planning
geometry_msgs/Pose start
geometry_msgs/Pose goal

# Distance between start and goal
float64 start_goal_dist

# Length of the path as calculated in ompl
float64 path_length

# Time for the planning plus all neccessary conversion between setting of goal till output of path as vector of PoseStamped
float64 total_planning_time
