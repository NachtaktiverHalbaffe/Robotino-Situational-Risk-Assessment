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

try:
    # Import when running from  roslaunch
    from risk_estimation.Environment import *
except:
    # Import when running as python script
    from Environment import *


class RoboEnv_gym(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self, config):
        self.actions_per_dimension = 5
        self.onesided = False
        self.config = config
        # create Environment
        visualize = True
        self.env_real = Environment(
            map_path=config["map_path"],
            relevant_segments=config["relevant_segments"],
            done_after_collision=config["done_after_collision"],
            adversary=None,
            visualize=visualize,
            start=config["start"],
            goal=config["goal"],
        )

        "some extra settings"
        self.n_reset_nodes = config["n_reset_nodes"]
        self.debug = False
        self.step_counter = 0
        self.episode_counter = 0
        self.visualize = False
        self.done = False

        "defining the action and observation space"
        self.action_space = spaces.MultiDiscrete([self.actions_per_dimension, self.actions_per_dimension])
        self.observation_space = spaces.Box(0, 255, shape=(1, 200, 200), dtype=np.uint8)

        "Here we are defining the actions and probabilites"  # TODO move this into the config file
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

    def reset(self):
        """This is the overwritable version of reset that links to the actual version

        Returns:
            [Dict of NumpyArrays]: Dictionary of the observation of the part and canvas
        """

        "Every n_reset_nodes episodes we want to generate new nodes to not overfit to the current ones"
        self.episode_counter = self.episode_counter + 1
        if (self.episode_counter % self.n_reset_nodes == 0) or self.episode_counter == 1:
            out = self.env_real.reset("adv1", new_nodes=True, start=self.config["start"], goal=self.config["goal"])
            observation, traj_vanilla = out
        else:
            observation, traj_vanilla = self.env_real.reset(
                "adv1", start=self.config["start"], goal=self.config["goal"]
            )

        "the observation should have greyscale fomat which is (1,200,200)"
        observation = np.expand_dims(observation, axis=0)
        return observation

    def step(self, network_action):
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
            print("expanding single dimensional action into two")

        "adding the actions to a dict that we are using to make sure that the Env stays compatible with the main file"
        actions_sb3 = {"angle_loc": self.angles[action_index_rot], "dist_loc": self.dists[action_index_dist]}
        probs_action_sb3 = {
            "angle_loc": self.angle_probs[action_index_rot],
            "dist_loc": self.dist_probs[action_index_dist],
            "combined": self.angle_probs[action_index_rot] * self.dist_probs[action_index_dist],
        }

        "calling the step function of the underlying Environment file, the passed actions are not used instead the passed action dict is"
        obs, reward, done, collision_status, adv1_node2, old_position, obstables_to_remove = self.env_real.step_adv1(
            0, 0, actions_sb3, probs_action_sb3
        )

        "packing the additonal info passes into the info dict required by gym standards"
        info = {"collision_status": collision_status, "old_position": old_position, "adv1_node2": adv1_node2}
        if done:
            info["final_state"] = collision_status
        if len(obstables_to_remove) > 0:
            info["obstables_to_remove"] = obstables_to_remove

        "the observation should have greyscale fomat which is (1,200,200)"
        obs = np.expand_dims(obs, axis=0)
        return (obs, reward, done, info)

    def close(self):
        pass

    def render(self, numpyarray):
        self.env_real.render(numpyarray)


if __name__ == "__main__":
    print("this wraps the real env into gym format so that it can be passed to SB3 or other rl libs")
    pass
