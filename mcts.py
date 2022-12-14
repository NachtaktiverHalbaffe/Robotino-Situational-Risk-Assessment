from ppo import *
import numpy as np

from probability_cal import real_to_pixel, action_prob_cal, risk_calculation, cumm_risk

ACTION_SPACE_STEP_ADVERSARY = 6
ACTION_SPACE_STEP_PROT = 4  # has to be even!


def choose_action(action_index):
    """
    Choosing the action to take in pixel

    Args:
        action_index (int): Action to take. It's the index from the action_space-list

    Returns:
        action_to_take (int): The action to take itself. It's a pixel value
    """
    action_to_take = real_to_pixel(action_index)
    return action_to_take


# to take the probability of the action choosen
prob_occurance = [0.08, 0.34, 0.1, 0.36, 0.1]


def action_prob(action_index):
    """
    Calculate the probability of the taken action

    Args:
        action_index (int): Action to take. It's the index from the action_space-list

    Returns:
        prob_occurance (int): The probability the action will occur
    """
    return prob_occurance[action_index]


class MonteCarloTreeSearch:
    """
    Tree exploration class

    Attributes:
        actions (int): The number of actions
        traj_vanilla (list(Node)): The unmanipulated trajectory
        environment (Environment, optional): Default environment. Defaults to None
        adv1 (optional): Adversary for which the tree search should be perfomed. Defaults to None
        observation (optional): The observation from the environment. Defaults to None
        test_mode (bool, optional): Training (False) or testing (True). Defaults to False
        adv ( optional): Defaults to False
    """

    def __init__(
        self,
        actions,
        traj_vanilla,
        environment=None,
        adv1=None,
        observation=None,
        test_mode=False,
        adv=None,
    ):
        # super().__init__()
        self.actions = actions
        """Number of actions"""
        self.traj_vanilla = traj_vanilla
        """Original trajectory"""
        self.probability = []
        """Probability of events of actions taken in a single episodes"""
        self.actions_taken = 0
        """Number of actions taken so far"""
        self.n_untried_actions = None
        """Untired list of actions taken only applicable if done_after_collision = true"""
        self.observation = observation
        """Default observation stays the same for complete tree"""
        self.test_mode = test_mode
        """Training or testing"""
        self.Nodes = len(traj_vanilla)
        """Number of nodes"""
        self.risk = []
        """Risk of each outcome"""
        self.edges = self.Nodes - 1
        """Number of edges correspond to series of actions"""
        self.env = environment
        """Default environment"""
        self.full_tree = None
        self.adv1 = adv1

    def untried_actions(self):
        """
        Provide a list of untried actions. Initially reset to 0,0,..

        Args:
            Takes the full tree from the class itself
            Takes the actions_taken list from the class itself

        Returns:
            self.n_untried_actions (list(int)): A list of the untried actions which remains
        """
        print("Series of actions taken: ", self.actions_taken)
        # n_untried_actions = self.full_tree[self.actions_taken:self.full_tree.shape[1]-1, self.actions_taken:self.full_tree.shape[1]-1]
        n_untried_actions = []
        for i in self.full_tree:
            n_untried_actions.append(
                i[self.actions_taken : self.full_tree.shape[1]].tolist()
            )
        self.n_untried_actions = np.array(n_untried_actions)
        return self.n_untried_actions

    def expand(self, binary_tree=True, new_edges=False, num_edges=0):
        """
        Expands/creates a tree. Steps:
        1. take actions: if success child_node = next_node
        2. check if next_node is a last node else next_node= traj_vanilla[+1]

        Args:
            binary_tree (bool, optional): If tree should be a binary tree. Defaults to True
            new_edges (bool, optional): If the edges exists already or have been already created. Defaults to False
            num_edges (int, optional): Number of edges in the tree. Only needed if new_edges==True. Defaults to 0

        Returns:
            final_actions (list(int)):
        """
        print("Expanding a tree...")
        # Get the number of edges
        if new_edges:
            edges = num_edges
        else:
            edges = self.Nodes - 1

        self.actions_taken = 0
        total_actions = ((self.actions - 1) * edges) + 1
        lst = []
        for i in range(edges - 1, -1, -1):
            # For each edge
            # lst.clear()
            # print("0's in step ", i , ": ", find_odd(i))
            lst.append([0] * self.find_odd(i))
            # print("1's in step ", i , ": ", 1)
            lst.append([1])
            # print("2's in step ", i , ": ", total_actions - find_odd(i)*2 -2)
            lst.append([2] * (total_actions - self.find_odd(i) * 2 - 2))
            # print("3's in step ", i , ": ", 1)
            lst.append([3])
            # print("4's in step ", i , ": ", find_odd(i))
            lst.append([4] * self.find_odd(i))
            # print(lst)

        final_actions = np.zeros((edges, total_actions), dtype=int)
        x, y = 0, 0
        for i in range(5 * edges):
            if i % 5 == 0 and i != 0:
                # Create new column in matrix
                x += 1
                y = 0
            for j in lst[i]:
                # Fill the columns with the values in lst
                final_actions[x][y] = j
                y += 1

        if binary_tree:
            final_actions = np.roll(final_actions, 1)
            final_actions[:, [2, int((total_actions + 1) / 2)]] = final_actions[
                :, [int((total_actions + 1) / 2), 2]
            ]
        self.full_tree = final_actions
        return final_actions

    def find_odd(self, n):
        """
        Args:
            n (int): Number to calculate

        Returns:
            2n+1
        """
        return n * 2 + 1

    def take_action(
        self, full_tree, done_after_collision=True, start_node=1, binary_tree=False
    ):
        """
        Takes the action in the environment. Determines if a collision occurs or not

        Args:
            full tree: The full tree on which the action is taken
            done_after_collision(bool, optional): If it should end after the first collision occurs. Defaults to True
            start_node (int, optional): Number of node from which it should start. Defaults to 2
            binary_tree (bool, optional): If it should create a binary tree. Defaults to False

        Returns:
            observation: Observation from the environment after action was taken
            reward (float): Reward based on the taken action
            collision_status (bool): If collision occured
            prob_collision (float, optional): Probability if a collision occured (number of collision normed by the size of the tree).\
                                                  Its rounded to 4 decimal places. Only returned if done_after_collision==False
            risk_total(float, optional): Product of cummulative risk and the probability of collision.\
                                                        Only returned if done_after_collision==False
        """
        n_collisions = 0
        n_successes = 0
        self.actions_taken = 0
        cumm_risk = 0

        if self.edges == 1:
            #####################################################
            # --------------------- Run without tree -------------------------
            #####################################################
            self.env.reset_traj(node_num=start_node)
            action_space = [0, 1, 2, 3, 4]
            self.actions_taken += 1

            for action_index in action_space:
                ###########################################
                # ---- Perform the action in the environment -----
                ###########################################
                pos_offset = choose_action(action_index)
                action_prob_value = action_prob(action_index)
                # observation, reward, done, collision_status, _, position_old = self.env.step_adv1(pos_offset,action_prob_value)

                action_angle_offset = np.deg2rad(
                    ACTION_SPACE_STEP_ADVERSARY * action_index
                    - (int(self.actions / 2) * ACTION_SPACE_STEP_ADVERSARY)
                )
                # Perform action itself
                (
                    observation,
                    reward,
                    done,
                    collision_status,
                    _,
                    old_position,
                ) = self.env.step_adv1(action_angle_offset, action_prob_value)

                # If collision occured, then increase collision counter, else increase success counter
                if collision_status:
                    n_collisions += 1
                    if done_after_collision == True:
                        return observation, reward, True, collision_status, _
                else:
                    n_successes += 1
                # Reset trajectory in environment
                self.env.reset_traj(node_num=start_node)

            # observation_, reward, done, collision_status, _, old_position = self.env.step_adv1(action_angle_offset, keep_searching=False)
            # return observation_, reward, True, collision_status, _
        else:
            ####################################################
            # ------------------- Perform with tree ------------------------
            ####################################################
            print("reset environment")
            if binary_tree:
                print("creating binary tree")
                tree_rows = full_tree.shape[1]

            for i in range(full_tree.shape[1]):
                # print("set previous environment")
                self.env.reset_traj(node_num=start_node)
                self.actions_taken += 1
                risk = 1
                observation = self.observation
                for j in range(full_tree.shape[0]):
                    ###########################################
                    # ---- Perform the action in the environment -----
                    ###########################################
                    action_index = full_tree[j][i]
                    # print("action index: ", action_index)
                    action_index_opt, prob, val, raw_probs = self.adv1.choose_action(
                        observation, test_mode=True
                    )
                    probs_all = np.round(raw_probs.cpu().detach().numpy().squeeze(0), 4)
                    # print(probs_all[action_index])
                    pos_offset = choose_action(action_index)
                    # Probability of occurance
                    action_prob_value = action_prob(action_index)
                    # action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(config['N_actions']/2)*ACTION_SPACE_STEP_ADVERSARY))
                    # observation, reward, done, collision_status, _, position_old = self.env.step_adv1(pos_offset,action_prob_value)

                    action_angle_offset = np.deg2rad(
                        ACTION_SPACE_STEP_ADVERSARY * action_index
                        - (int(self.actions / 2) * ACTION_SPACE_STEP_ADVERSARY)
                    )
                    (
                        observation,
                        reward,
                        done,
                        collision_status,
                        _,
                        old_position,
                    ) = self.env.step_adv1(action_angle_offset, action_prob_value)

                    po_pc = probs_all[action_index] * action_prob_value
                    risk *= action_prob_value
                    # a = input()

                    # If collision occured, then increase collision counter, else increase success counter
                    if collision_status:
                        n_collisions += 1
                        if done_after_collision == True:
                            return observation, reward, True, collision_status, _
                        break
                    elif done and not collision_status:
                        n_successes += 1
                        break
                # print("Risk: ", risk)
                # Calculate cummulative risk
                cumm_risk += risk
                risk = 1
            # a = input()

        #########################################
        #  -------------- Risk calculation -----------------
        #########################################
        risk_avg = cumm_risk / full_tree.shape[1]
        prob_collision = n_collisions / full_tree.shape[1]
        risk_total = cumm_risk * prob_collision

        # print("Cummulative Risk: ", cumm_risk)
        # print("Average Risk: ", cumm_risk/full_tree.shape[1])
        # print("Total Risk: ", cumm_risk*(n_collisions/full_tree.shape[1]))
        # print("Total Collisions: ", n_collisions)
        # print("Total Successes: ", n_successes)
        # print("Probability of being able to reach: ", n_successes/full_tree.shape[1])
        # print("Probability of Collision: ", n_collisions/full_tree.shape[1])
        return (
            observation,
            reward,
            True,
            collision_status,
            _,
            np.round(prob_collision, 4),
            risk_total,
        )

    def is_last_node(self):
        """Check if its a last node

        Check last node values and compare with traj_vanilla last node or  check if total steps taken == length of traj_vanilla

        Returns:
            True if it is last node or False if it isn't last node
        """
        if (self.steps == len(self.traj_vanilla)) or (
            self.child_node == self.traj_vanilla[len(self.traj_vanilla) - 1]
        ):
            return True
        else:
            return False

    def is_fully_expanded(self):
        """
        Check if its a fully expanded and all the actions are taken

        Returns:
            True if it's fully expanded or False if it's not
        """
        if self.actions_taken == self.full_tree.shape[1]:
            return True
        else:
            return False

    def best_child(self):
        """Return the best child with max probability. Currently not implemented"""
        pass

    def best_tree(self):
        """Return the smallest tree without collision. Currently not implemented"""
        pass

    def longest_tree(self):
        """Return the longest tree without collision check probability. Currently not implemented"""
        pass

    def best_action(self):
        """Return the best possible action. Currently not implemented"""
        pass

    def _tree_policy(self):
        """self node to run rollout policy. Currently not implemented"""
        pass

    def action_probability(self):
        """return the probability of certain action. Currently not implemented"""
        pass

    def create_children(self, env, config, action_index):
        """
        Performs all action in the environment N_actions times (specified in config file)\
        or until one collision occurs

        Returns:
            action_index (int): The action to take in form of an index which indicates the \
                                            position of the action in the list of the action space
            observation_: Oberservation from the environment after the collision provoking action or last action was taken 
            reward (float): Reward after the collision provoking action or last action was taken 
            done (bool): after the collision provoking action or last action was taken 
            collision_status (bool): If collision occured
        """
        # print("in create children")
        action = 0
        while action < config["N_actions"]:
            # print(action)
            action_index_ida = action

            # t3 = time.perf_counter()
            action_angle_offset = np.deg2rad(
                ACTION_SPACE_STEP_ADVERSARY * action_index_ida
                - (int(config["N_actions"] / 2) * ACTION_SPACE_STEP_ADVERSARY)
            )

            (
                observation_,
                reward,
                done,
                collision_status,
                _,
                position_old,
            ) = env.step_adv1(action_angle_offset, create_leaf_node=False)
            # step_total_time += time.perf_counter() - t3

            """ Debugging code"""
            if collision_status == 1:
                # print("Action Index, collision status: ", action_index_ida, collision_status)
                # break
                action_index = action
                action += 1
                return action_index, observation_, reward, done, collision_status
            else:
                action += 1

        return action_index, observation_, reward, done, collision_status

    # TODO unused?
    def expand_tree(self, env, config, action_index):
        """ """
        # print("in expand tree")
        (
            action_index,
            observation_,
            reward,
            done,
            collision_status,
        ) = self.create_children(env=env, config=config, action_index=action_index)

        return action_index, observation_, reward, done, collision_status

    def find_best_child(
        self, raw_probs, positions, node_num=1, prev_prob=0, pos_prev=[]
    ):
        """

        Args:
            raw_probs: The raw probabilities in which it is searched
            positions:
            node_num (int, optional): Defaults to 1
            prev_prob (int, optional): Defaults to 0
            pos_prev (list, optional): Defaults to empty list

        Returns:
            action_index:
            prev_prob:
            m_node_num:
            pos_prev:
        """
        temp_prev = prev_prob
        m_node_num = node_num
        # print("Find best raw probs: ", raw_probs)
        for n in range(len(raw_probs)):
            """Calculation 2nd max probability"""
            prob_act = raw_probs[n]
            indices = np.argsort(prob_act)
            prob_act.sort()

            if prob_act[3] > prev_prob:
                action_index = indices[3]
                prev_prob = prob_act[3]
                pos_prev = positions[n]
                m_node_num = n + node_num

            if temp_prev == prev_prob:
                action_index = 2

        return action_index, prev_prob, m_node_num, pos_prev

    def __version__(self):
        print("IAS_AR_0.22.1")


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
