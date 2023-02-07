from stable_baselines3 import PPO
import numpy as np
from stable_baselines3.common.policies import obs_as_tensor


class Sb3_as_adv:
    def __init__(self, env, load_dir):
        self.env = env
        "Instantiate the agent"
        self.model = PPO.load(load_dir, env)

    def choose_action(self, obs, test_mode=True):
        """This is to make sure that the output that is needed"""
        "if the obseration is not in image format (1,200,200) expand the first dimension"
        if obs.ndim == 2:
            obs = np.expand_dims(obs, axis=0)
        "pass the observastion to the agent"
        action, _ = self.model.predict(obs)
        "expand it as a batch of size 1"
        obs = np.expand_dims(obs, axis=0)
        "pass to gpu"
        obs = obs_as_tensor(obs, self.model.policy.device)

        "obtain distribution and get the probabilities of the individual actions"
        dis = self.model.policy.get_distribution(obs)
        probs_dist = dis.distribution[0].probs
        probs_angle = dis.distribution[1].probs

        "here we change from individual probabilites to a combined probability"
        probs_np_dist = probs_dist.detach().cpu().numpy()
        probs_np_angle = probs_angle.detach().cpu().numpy()
        probs = np.dot(probs_np_dist.transpose(), probs_np_angle)
        probs = probs.flatten()

        "here we convert it into a 1d action for the mcts"
        action_of_81 = int(action[0] * len(probs_np_dist) + action[1])

        return action_of_81, {}, {}, probs
