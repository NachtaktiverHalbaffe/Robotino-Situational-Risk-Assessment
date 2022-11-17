from ppo import *
import numpy as np

ACTION_SPACE_STEP_ADVERSARY = 5
ACTION_SPACE_STEP_PROT = 4  # has to be even!

class MonteCarloTreeSearch:
    def __init__(self, actions, traj_vanilla, environment=None, observation=None, test_mode=False, adv=None):
        #super().__init__()
        self.actions = actions # Number of actions
        self.traj_vanilla = traj_vanilla    # Original trajectory
        self.probability = []   # Probability of events of actions taken in a single episodes
        self.actions_taken = 0 # Number of actions taken so far
        self.n_untried_actions = None # Untired list of actions taken only applicable if done_after_collision = true
        self.observation = observation  # Default observation stays the same for complete tree
        self.test_mode = test_mode  # Training or testing
        self.Nodes = len(traj_vanilla)  # Number of nodes
        self.risk = []      # Risk of each outcome
        self.edges = self.Nodes -1 # Number of edges correspond to series of actions
        self.env = environment # Default environment
        self.full_tree = None

    def untried_actions(self):
        print("Series of actions taken: ", self.actions_taken)
        """ Provide a list of untried actions intially reset to 0,0,.."""
        #n_untried_actions = self.full_tree[self.actions_taken:self.full_tree.shape[1]-1, self.actions_taken:self.full_tree.shape[1]-1]
        n_untried_actions = []
        for i in self.full_tree:
            n_untried_actions.append(i[self.actions_taken:self.full_tree.shape[1]].tolist())
        self.n_untried_actions = np.array(n_untried_actions)
        return self.n_untried_actions

    def expand(self, corner_cases=True, new_edges=False, num_edges=0):
        print("Expanding a tree...")
        """ expand a tree"""
        """ steps
            take actions: if success child_node = next_node
            check if next_node is a last node
            else next_node= traj_vanilla[+1]"""
        if new_edges:
            edges = num_edges
        else:
            edges = self.Nodes- 1
        self.actions_taken = 0
        total_actions = ((self.actions-1) * edges) + 1
        lst = []
        for i in range(edges-1, -1, -1):
            #lst.clear()
            #print("0's in step ", i , ": ", find_odd(i))
            lst.append([0] * self.find_odd(i))
            #print("1's in step ", i , ": ", 1)
            lst.append([1])
            #print("2's in step ", i , ": ", total_actions - find_odd(i)*2 -2)
            lst.append([2] * (total_actions - self.find_odd(i)*2 -2))
            #print("3's in step ", i , ": ", 1)
            lst.append([3])
            #print("4's in step ", i , ": ", find_odd(i))
            lst.append([4] * self.find_odd(i))
            #print(lst)

        final_actions = np.zeros((edges,total_actions), dtype=int)
        x,y = 0,0    
        for i in range(5*edges):
            if i%5 ==0 and i!=0:
                #print("New column")
                x+=1
                y= 0
            for j in lst[i]:
                #print(j)
                final_actions[x][y] = j
                y+=1
        if corner_cases:
            final_actions = np.roll(final_actions, 1)
            final_actions[:,[2, int((total_actions+1)/2)]] = final_actions[:,[int((total_actions+1)/2), 2]]
        self.full_tree = final_actions
        return final_actions

    def find_odd(self,n):
        return (n*2 + 1)

    def take_action(self,full_tree, done_after_collision=True, start_node=1, binary_tree=False):
        """ Return if its a collision or success"""
        n_collisions = 0
        n_successes = 0
        self.actions_taken = 0
        if self.edges == 1:
            self.env.reset_traj(node_num=start_node)
            action_space = [0,1,2,4,5]
            self.actions_taken +=1 
            for action_index in action_space:
               # print("action index: ", action_index)
                """ Step Function"""
                action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(self.actions/2)*ACTION_SPACE_STEP_ADVERSARY))
                observation_, reward, done, collision_status, _ = self.env.step_adv1(action_angle_offset, create_leaf_node=False, keep_searching=False)
                if collision_status:
                    n_collisions+=1
                    if done_after_collision == True:
                        return observation_, reward, True, collision_status, _
                else:
                    n_successes+=1
            observation_, reward, done, collision_status, _ = self.env.step_adv1(action_angle_offset, keep_searching=False)
                #return observation_, reward, True, collision_status, _ 
        else:
            print("reset environment")

            if binary_tree:
                print("creating binary tree")
                for i in range(2):
                #print("set previous environment")
                    self.env.reset_traj(node_num=start_node)
                    self.actions_taken +=1 
                    for j in range(full_tree.shape[0]):
                        action_index = full_tree[j][i]
                        #print("action index: ", action_index)
                        action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(self.actions/2)*ACTION_SPACE_STEP_ADVERSARY))
                        observation_, reward, done, collision_status, _ = self.env.step_adv1(action_angle_offset, keep_searching=False)
                        if collision_status:
                            n_collisions+=1
                            if done_after_collision == True:
                                return observation_, reward, True, collision_status, _
                            break
                        elif done and not collision_status:
                            n_successes+=1

                    
                
            for i in range(full_tree.shape[1]):
                #print("set previous environment")
                self.env.reset_traj(node_num=start_node)
                self.actions_taken +=1 
                for j in range(full_tree.shape[0]):
                    action_index = full_tree[j][i]
                    #print("action index: ", action_index)
                    action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(self.actions/2)*ACTION_SPACE_STEP_ADVERSARY))
                    observation_, reward, done, collision_status, _ = self.env.step_adv1(action_angle_offset, keep_searching=False)
                    #a = input()
                    if collision_status:
                        n_collisions+=1
                        if done_after_collision == True:
                            return observation_, reward, True, collision_status, _
                        break
                    elif done and not collision_status:
                            n_successes+=1
                
        print("Total Collisions: ", n_collisions)
        print("Total Successes: ", n_successes)
        print("Probability of being able to reach: ", n_successes/full_tree.shape[1])
        print("Probability of Collision: ", n_collisions/full_tree.shape[1])
        return observation_, reward, True, collision_status, _
        

    def is_last_node(self):
        """ Check if its a last node"""
        """ Check last node values and compare with traj_vanilla last node or 
            check if total steps taken == length of traj_vanilla"""
        if (self.steps == len(self.traj_vanilla)) or (self.child_node == self.traj_vanilla[len(self.traj_vanilla)-1]):
            return True
        else:
            return False

    def is_fully_expanded(self):
        """ Check if its a fully expanded and all the actions are taken"""
        if self.actions_taken == self.full_tree.shape[1]:
            return True
        else:
            return False

    def best_child(self):
        """ Return the best child with max probability"""
        pass
    
    def best_tree(self):
        """ Return the smallest tree without collision"""
        pass

    def longest_tree(self):
        """ Return the longest tree without collision check probability"""
        pass

    def best_action(self):
        """ Return the best possible action"""
        pass

    def _tree_policy(self):
        """ self node to run rollout policy"""
        pass

    def action_probability(self):
        """ return the probability of certain action"""
        pass

# def main():
#     actions = 5
#     traj_vanilla = [(62, 74), (67, 81), (73,99), (87,110), (104,118), (109,125)]
#     mcts = MonteCarloTreeSearch(actions, traj_vanilla,None, None, True)
#     action_space = mcts.expand()
#     print(action_space)
#     mcts.take_action(action_space)

#     """ best node with least collision
#         best action with least collision
#         worst actions with highest collisions"""

# if __name__ == '__main__':
#     main()    