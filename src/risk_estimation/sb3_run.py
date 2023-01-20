#
# Created on: Tue Okt 26 2021
#
# Description: runs the sheet nesting environment with agents from SB3
#
# Author: Binder, Kai
#
import os
import time
import gym
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.policies import ActorCriticCnnPolicy
from stable_baselines3 import PPO, TD3, SAC, A2C
from stable_baselines3.common.monitor import Monitor
from sb3_model_same import SameExtractor
import env_gym #This is greyed out but is needed for gym to load the env from here
import config
import multiprocessing

try:
    from sb3_callbacks import TrainPlotsCallback
    plot_q_values = True
except ModuleNotFoundError:
    plot_q_values = False


def linear_schedule(initial_value: float):
    """
    Linear learning rate schedule.

    :param initial_value: Initial learning rate.
    :return: schedule that computes
      current learning rate depending on remaining progress
    """
    def func(progress_remaining: float) -> float:
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        return progress_remaining * initial_value

    return func

def test_run(configs,plot_dir='throw_away', env_name='robo_navigation-v0', n_envs=4, learning_rate=0.0001, steps = 50_000):
    """runs the sheet nesting environment with agents from SB3,
       must be run from __main__ wrapper for thread safety

    Args:
        plot_dir (String): name of Folder results are saved in
        env_name (String): gym registration name of the env to use
        n_env (int): number of environments to run
        learning_rate: learning_rate of the agent
    """
    learning_rate = learning_rate
    print('Making Environment')
    # Begin making the Environment,  change the number in range to
    # set amount. Use SubprocVec if multiprocessing is needed
    env = SubprocVecEnv([lambda: Monitor(gym.make(env_name, config=configs))
                         for i in range(n_envs)])
    # env = DummyVecEnv([lambda: Monitor(gym.make(env_name, config=configs))
    #                      for i in range(1)])
    print(env.num_envs,'Environments created')
    env.reset()
    # Defining the feature extractor to use
    policy_kwargs = dict(
        features_extractor_class=SameExtractor)
    print('Making Agent')
    model = PPO(ActorCriticCnnPolicy, env, policy_kwargs=policy_kwargs,
                verbose=1, tensorboard_log='./logs/',
                n_steps=64, learning_rate=learning_rate, device='cuda')

    if plot_q_values:
        # Callbacks are functions called during training.
        # These are used for logging purpuses
        train_plots = TrainPlotsCallback(plot_loc=plot_dir)
        # save 10 models during the training process
        for i in range(10):
            model.learn(total_timesteps=10_000,callback=train_plots)
            model.save(f"{plot_dir}/model_{i*steps/10}_timesteps")
    else:
        for i in range(10):
            model.learn(total_timesteps=10_000)
            model.save(f"{plot_dir}/model_{i*steps/10}_timesteps")

if __name__ == '__main__':
    start = time.time()
    # Where results of the run will be saved. Running twice with the same name causes overwrite
    load_dir = None
    # Current Options for the Env are 'robo_navigation-v0'
    env_name = 'robo_navigation-v0'
    # number of environments to use
    n_envs = int(multiprocessing.cpu_count()/2)
    use_test = False
    use_custum_extractor = False
    configs = config.configs[0]
    learning_rate = linear_schedule(0.00001)
    plot_dir = 'testing_env_health'
    steps = 5_000
    
    test_run(configs=configs,
             plot_dir=plot_dir, env_name=env_name,
             learning_rate =learning_rate,
             steps = steps)
             
    end = time.time()
    print(end-start)
