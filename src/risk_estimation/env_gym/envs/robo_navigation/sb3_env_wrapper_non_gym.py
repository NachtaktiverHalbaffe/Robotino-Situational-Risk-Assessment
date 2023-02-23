#
# Created on: Tue Nov 2 2021
#
# Description: an environment is a class with a minmum of 4 functions
# 1. initialization function (sets the initial state)
# 2. step function, take an action variable, return list of four things:
#   - the next state, the reward for the current state, a stop-boolean, optional render on our problem
# 3. reset function, which resets the state to the start state
# 4. render, gives relevant information about the behavior of our agent by drawing the canvas with the placed parts
#
# all other things are internal utility functions, not for the agent
# Reference:
# https://github.com/openai/gym/blob/master/docs/creating-environments.md
#
#
# Author: Binder, Kai
import gym
from gym import spaces
import numpy as np
from PIL import Image

try:
    # Import when running from  roslaunch
    from risk_estimation.Environment import *
    from utils.risk_estimation_utils import (
        loadErrorDistribution,
        loadErrorDistributionLIDAR,
    )
except:
    # Import when running as python script
    from Environment import *
    from utils.risk_estimation_utils import (
        loadErrorDistribution,
        loadErrorDistributionLIDAR,
    )

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))


class RoboEnv_gym_2(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        config,
        env=None,
        obstacles=None,
        invertMap=False,
        isTraining=True,
        errorDistrDistPath=None,
        errorDistrAnglePath=None,
        useLidar=False,
    ):
        self.persisten_map = False
        self.actions_per_dimension = 5
        self.num_action_options = self.actions_per_dimension
        self.onesided = False
        self.config = config
        # create Environment
        visualize = True

        if obstacles == None:
            self.env_real = Environment(
                map_path=config["map_path"],
                relevant_segments=config["relevant_segments"],
                done_after_collision=config["done_after_collision"],
                adversary=None,
                visualize=visualize,
                start=config["start"],
                goal=config["goal"],
                invertMap=invertMap,
                isTraining=isTraining,
            )
        else:
            self.env_real = Environment(
                map_path=config["map_path"],
                relevant_segments=config["relevant_segments"],
                done_after_collision=config["done_after_collision"],
                adversary=None,
                visualize=visualize,
                start=config["start"],
                goal=config["goal"],
                obstacles=obstacles,
                invertMap=invertMap,
                isTraining=isTraining,
            )
        "some extra settings"
        self.n_reset_nodes = config["n_reset_nodes"]
        self.debug = False
        self.step_counter = 0
        self.episode_counter = 0
        self.visualize = False
        self.done = False

        "defining the action and observation space"
        self.action_space = spaces.MultiDiscrete(
            [self.actions_per_dimension, self.actions_per_dimension]
        )
        self.observation_space = spaces.Box(0, 255, shape=(1, 200, 200), dtype=np.uint8)

        "Here we are defining the actions and probabilites"
        # Use hard-coded/experimental data
        if self.actions_per_dimension == 9:
            self.angles = [-0.28, -0.21, -0.14, -0.07, 0.0, 0.07, 0.14, 0.21, 0.28]
            self.angle_probs = [
                0.01395939 / 2,
                0.04060914 / 2,
                0.12817259 / 2,
                0.21700508 / 2,
                0.57614213,
                0.21700508 / 2,
                0.12817259 / 2,
                0.04060914 / 2,
                0.01395939 / 2,
            ]
            self.dists = np.array([-9, -7, -5, -3, 0, 3, 5, 7, 9]) * 0.05
            self.dist_probs = [
                0.10135135 / 2,
                0.17567568 / 2,
                0.30135135 / 2,
                0.23108108 / 2,
                0.06621622,
                0.23108108 / 2,
                0.30135135 / 2,
                0.17567568 / 2,
                0.10135135 / 2,
            ]
        else:
            self.angles = [-0.24, -0.12, 0.0, 0.12, 0.24]
            self.angle_probs = [
                0.01395939 / 2 + 0.04060914 / 2,
                0.12817259 / 2 + 0.21700508 / 2,
                0.57614213,
                0.21700508 / 2 + 0.12817259 / 2,
                0.04060914 / 2 + 0.01395939 / 2,
            ]
            self.dists = np.array([-8, -4, 0, 4, 8]) * 0.05
            self.dist_probs = [
                0.10135135 / 2 + 0.17567568 / 2,
                0.30135135 / 2 + 0.23108108 / 2,
                0.06621622,
                0.23108108 / 2 + 0.30135135 / 2,
                0.17567568 / 2 + 0.10135135 / 2,
            ]
        if not useLidar:
            # load dynamically from CSV file
            if errorDistrDistPath != None:
                self.dists, self.dist_probs = loadErrorDistribution(
                    errorDistrDistPath, bins=self.actions_per_dimension
                )
            if errorDistrAnglePath != None:
                self.angles, self.angle_probs = loadErrorDistribution(
                    errorDistrAnglePath, bins=self.actions_per_dimension
                )
        else:
            self.angles, self.angle_probs = loadErrorDistributionLIDAR()

    def set_test(self) -> None:
        """Sets the environment to a test env"""
        self.train_env = False

    def set_persisten_map(self, persisten_map=False):
        self.persisten_map = persisten_map

    def reset(
        self, agent_name="adv1", new_nodes=False, start=[62, 74], goal=[109, 125]
    ):
        """This is the overwritable version of reset that links to the actual version

        Returns:
            [Dict of NumpyArrays]: Dictionary of the observation of the part and canvas
        """
        observation, traj_vanilla = self.env_real.reset(
            agent_name,
            new_nodes=new_nodes,
            start=start,
            goal=goal,
            persisten_map=self.persisten_map,
        )
        return observation  # , traj_vanilla

    def step(self, network_action, action_prob_placeholder=0):
        return self.step_adv1(
            network_action, action_prob_placeholder=action_prob_placeholder
        )

    def step_adv1(self, network_action, action_prob_placeholder=0):
        """This splits the 25 actions into rotation dist pairs

        Args:
            network_action (np.array): The action choosen by the agent
        Returns:
            [tuple]: standard rl environment return
        """
        # interpreting the network action
        if isinstance(network_action, np.ndarray):
            action_index_dist, action_index_rot = network_action
        else:
            action_index_rot = int(network_action % self.actions_per_dimension)
            action_index_dist = int(network_action / self.actions_per_dimension)

        "adding the actions to a dict that we are using to make sure that the Env stays compatible with the main file"
        actions_sb3 = {
            "angle_loc": self.angles[action_index_rot],
            "dist_loc": self.dists[action_index_dist],
        }
        probs_action_sb3 = {
            "angle_loc": self.angle_probs[action_index_rot],
            "dist_loc": self.dist_probs[action_index_dist],
            "combined": self.angle_probs[action_index_rot]
            * self.dist_probs[action_index_dist],
        }

        ################################################################################
        ## This is currently setting the action of the distance change to 0 change    ##
        ## This is done because the brute force is also currently only using 0 change ##
        ################################################################################
        actions_sb3 = {"angle_loc": self.angles[action_index_rot], "dist_loc": 0}
        probs_action_sb3 = {
            "angle_loc": self.angle_probs[action_index_rot],
            "dist_loc": 1,
            "combined": self.angle_probs[action_index_rot] * 1,
        }

        "calling the step function of the underlying Environment file, the passed actions are not used instead the passed action dict is"
        (
            obs,
            reward,
            done,
            collision_status,
            adv1_node2,
            old_position,
            obstables_to_remove,
        ) = self.env_real.step_adv1(0, 0, actions_sb3, probs_action_sb3)

        "packing the additonal info passes into the info dict required by gym standards"
        info = {
            "collision_status": collision_status,
            "old_position": old_position,
            "adv1_node2": adv1_node2,
        }
        if done:
            info["final_state"] = collision_status
        if len(obstables_to_remove) > 0:
            info["obstables_to_remove"] = obstables_to_remove

        "the observation should have greyscale fomat which is (1,200,200)"
        obs = np.expand_dims(obs, axis=0)
        return (obs, reward, done, info)

    def get_actions(self, network_action):
        "convert a network action into a the corresponding real world actions"
        if isinstance(network_action, np.ndarray):
            action_index_dist, action_index_rot = network_action
        else:
            action_index_rot = int(network_action % self.num_action_options)
            action_index_dist = int(network_action / self.num_action_options)
        return self.angles[action_index_rot], self.dists[action_index_dist]

    def action_prob(self, network_action):
        "give the probability of a network action based on the distribution"
        if isinstance(network_action, np.ndarray):
            action_index_dist, action_index_rot = network_action
        else:
            action_index_rot = int(network_action % self.num_action_options)
            action_index_dist = int(network_action / self.num_action_options)
        return self.angle_probs[action_index_rot] * self.dist_probs[action_index_dist]

    def reset_traj(self, node_num=1, pos=[-1, -1]):
        "passing to the methode of the name in the real environment"
        self.env_real.reset_traj(node_num=node_num, pos=pos)

    def close(self):
        pass

    def render(self, numpyarray):
        self.env_real.render(numpyarray)


if __name__ == "__main__":
    print(
        "this wraps the real env into gym format so that it can be passed to SB3 or other rl libs"
    )
    pass
