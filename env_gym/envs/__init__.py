import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)


# Nesting
# ----------------------------------------
kwargs_robo = {"config": {}}

register(
    id="robo_navigation-v0",
    entry_point="env_gym.envs.robo_navigation:RoboEnv_gym",
    kwargs=kwargs_robo,
    max_episode_steps=1000,
    reward_threshold=2000000,
)

register(
    id="robo_navigation-single_dimension-v0",
    entry_point="env_gym.envs.robo_navigation:RoboEnv_gym_2",
    kwargs=kwargs_robo,
    max_episode_steps=1000,
    reward_threshold=2000000,
)

