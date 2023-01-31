# Probabilities calculated by the reinforcement learning agents
float32[] probs_rl
# Probabilities calculated by brute-forcing => Can be taken as ground truth
float32[] probs_brute
# The corresponding trajectory for the collisions probabilities
nav_msgs/Path[] trajectories
# Number of nodes of the trajectory
int16[] nr_nodes
# Length of trajectory
float32[] length_traj
# The actions taken by the adversary to provoke a crash
std_msgs/Int16MultiArray action_to_crash
# Number of wall discards
int32[] wall_discards
