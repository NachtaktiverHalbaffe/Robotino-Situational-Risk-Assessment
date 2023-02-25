from copy import deepcopy
import os, sys
import time
import gym
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle as pkl
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.policies import ActorCriticCnnPolicy
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from PIL import Image, ImageDraw
from gym.envs.registration import register

try:
    import risk_estimation.env_gym
    from .config import configs
    from .SaveData import save_data, traj_length
    from .mcts import *
    from .sb3_as_adv import Sb3_as_adv
    from .sb3_model_same import SameExtractor
    from utils.constants import Paths
except:
    import env_gym
    from config import configs
    from SaveData import save_data, traj_length
    from risk_estimation.mcts import *
    from sb3_as_adv import Sb3_as_adv
    from sb3_model_same import SameExtractor

# TODO add better control of the trajectories
# TODO brute force using one dimension, only use the ANGLE that is the domninant side
# TODO MODIFY the env single dimsension
# TODO MODIFY the monte carlo
PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))
sys.path.append(f"{PATH}/src/risk_estimation/sb3_model_same.py")
# kwargs_robo = {"config": {}}
# register(
#     id="robo_navigation-single_dimension-v0",
#     entry_point="risk_estimation.env_gym.envs.robo_navigation:RoboEnv_gym_2",
#     kwargs=kwargs_robo,
#     max_episode_steps=1000,
#     reward_threshold=2000000,
# )


def calculate_collosion_order(prob_collision_with_Node):
    """
    This function takes in a list of tuples as input, where each tuple contains two values, a float value and an integer.
    The float value represents the probability of collision with a specific node and the integer value represents the order of appearance of the node.
    The function sorts the list of tuples based on the order of appearance and calculates the collision probability by iterating through the sorted list.
    It returns a single float value representing the overall collision probability.
    """
    sorted_by_appearence = sorted(prob_collision_with_Node, key=lambda tup: tup[1])
    remaining_paths = 1
    crash_probabilty = 0
    for prob in sorted_by_appearence:
        remain_prob = remaining_paths * prob[0]
        crash_probabilty = crash_probabilty + remain_prob
        remaining_paths = remaining_paths * (1 - prob[0])
    return crash_probabilty


def modify_map_out(
    map_ref, obstacles_org, obstacles, color=(255, 255, 255), convert_back_to_grey=True
):
    """
    This is takes a set of obstacles to remove from the image and a set to place in the image, often used for shifting obstacles in
    the map by passing the same obstacles at the old and new different locations

    Args:
        map_ref (Image): The map reference (image) we want to modify
        obstacles_org (set(Obstacle)): Obstacles to remove from the image
        obstacle (set(Obstacle)): Obstacles to place in the image
        color( set(int,int,int), optional): the color of the object being placed (in RGB). Defaults to white (255,255,255)
        convert_back_to_grey (bool, optional): if the image should be converted back into grey, needed if used in the PRM. Defaults to true

    Returns:
        map_ref_adv (Image): the modified image
    """
    map_ref_adv = deepcopy(map_ref)
    map_ref_adv = map_ref_adv.convert("RGB")
    map_ref_adv_draw = ImageDraw.Draw(map_ref_adv)
    add = [(2, 2), (2, -2), (-2, -2), (-2, 2)]
    a = 0
    add = [(a, a), (a, -a), (-a, -a), (-a, a)]
    # add = [(0,0),(0,-0),(-0,-0),(-0,0)]
    for obstacle in obstacles_org:
        # cv2.fillConvexPoly(self.map_ref_adv,obstacle.corners, color='black')
        # increase the size of the obstacle by one pixel
        # corners = [tuple(map(lambda i,j:i+j,obstacle.corners[i],add[i])) for i in range(4)]
        map_ref_adv_draw.polygon(obstacle.corners, fill=(0, 0, 0), outline=(0, 0, 0))
    # add = [(1,1),(1,-1),(-1,-1),(-1,1)]
    for obstacle in obstacles:
        # cv2.fillConvexPoly(self.map_ref_adv,obstacle.corners, color='white')
        # corners = [tuple(map(lambda i,j:i+j,obstacle.corners[i],add[i])) for i in range(4)]
        map_ref_adv_draw.polygon(obstacle.corners, fill=color, outline=color)
    if convert_back_to_grey:
        map_ref_adv = map_ref_adv.convert("L")
    return map_ref_adv


def run_pruned(env, adv1, crash_point=0, reduce=False, forced_traj=None):
    """
    Args:
        env: An environment object
        adv1: An agent object
        crash_point: An integer representing the point at which the collision occurs. default value is 0.
        reduce: A boolean value indicating whether the persistent map is reduced. default value is False.
        forced_traj: A potential forced trajectory for the environment. default value is None which means vanilla is used.

    Returns:
        prob_collision(float) : Probability of collision
    """
    observation, traj_vanilla = env.env_real.reset(
        "adv1",
        start=[93, 122],
        goal=configs[0]["goal"],
        forced_traj=forced_traj,
        persisten_map=reduce,
    )
    obeservation_orig = observation
    mcts = MonteCarloTreeSearch(
        configs[0]["N_actions"],
        traj_vanilla,
        env,
        adv1,
        obeservation_orig,
        test_mode=True,
    )
    pruned_tree = mcts.expand()
    (
        observation_,
        reward,
        done,
        collision_status,
        _,
        prob_collision,
        risk_ep,
    ) = mcts.take_action(pruned_tree, done_after_collision=False, use_new_agent=True)
    return prob_collision


def run_crash_and_remove(
    configs,
    env_name,
    use_brute_force_baseline=False,
    save_location=Paths.RISK_ESTIMATION.value,
    replay_on=False,
    load_location=None,
    attempts=10,
    expand_length=3,
    amount_of_exploration=10,
    initialTraj=None,
    start=None,
    goal=None,
    obstacles=None,
    invertMap=False,
    errorDistrDistPath=None,
    errorDistrAnglePath=None,
    useLidar=False,
):
    """
    This function is used to run a simulation of an agent navigating through an environment, with the goal of
    crashing in in the environment. After a crash the crashed into Object is removed and search for other potential
    crashes. The potential crashes are then evaluated with a small brute force expansion to obtain the
    risk of the trajectory.
    It uses a combination of a PPO agent and a Brute force agent to calculate the risk.

    Args:
        configs (dict): a dictionary containing configurations for the environment and the agents
        env_name (str): a string representing the name of the environment to be used, gym uses this to load the env
        use_brute_force_baseline (bool, optional): a boolean indicating whether to use brute force as a baseline. Defauls to False
        save_location (str, optional): filepath to save the results
        replay_on (bool, optional): Indicating whether to replay a previous simulation. Defaults to False
        load_location (str, optional): Filepath to load previous results. Defaults to None
        attempts (int, optional): Number of trajectories to simulation and evaluate. Defaults to 10
        expand_length (int, optional): Length of the brute for search around a collision. Defaults to 3
        amount_of_exploration (int, optional): Number of attemts the agent gets to find a collision. Defaults to 10
        intialTraj( list of coordinates, optional): Initial trajectory for which the risk estimation should be run. Defaults to None
        obstacles (list(Obstacle), optional): Detected obstacles which should be used in probability estimation
        invertMap(bool, optional): If map reference should be inverted. Needed when a PRM is used
        errorDistrDistPath (str, optional): Path to the CSV if own error distribution data for distance should be used
        errorDistrAnglePath (str, optional): Path to the CSV if own error distribution data for angles should be used
        useLidar (bool, optional): If LIDAR error distribution should be used for probability estimation. The agent is not trained for this, but should kinda work

    Returns (in a dict):
        rl_prob (list(float)): Probabilities calculated by the reinforcement learning agents
        brute_prob (list(float)): Probabilities calculated by brute-forcing. Can be taken as an ground truth
        traj (list(Trajectory)): Trajectory used in the corresponding risk estimation
        len_traj (list(int)): Number of nodes of the trajectory
        len_traj_list_in_m  (list()): Actual length of the trajectory
        rl_actions_to_crash_lists (list of list(int)): A list the adversary has done to provoke a collision
        wall_discards (list(int)): Number of discarded RL runs due to wall collision
        collided_obs (list(Obstacle)): A list of the obstacles with which the roboter collided in the fault injector
    """

    #  -------------------------- loading the agent and the environment ----------------------------------------
    env_name = "robo_navigation-single_dimension-v0"
    # env = DummyVecEnv([lambda: Monitor(gym.make(env_name, config=configs))
    #                  for i in range(1)])

    if obstacles != None:
        env = gym.make(
            env_name,
            config=configs,
            obstacles=obstacles,
            invertMap=invertMap,
            isTraining=False,
            errorDistrAnglePath=errorDistrAnglePath,
            errorDistrDistPath=errorDistrDistPath,
            useLidar=useLidar,
        )
        # env = gym.make(env_name, config=configs)
    else:
        env = gym.make(
            env_name,
            config=configs,
            invertMap=invertMap,
            isTraining=False,
            errorDistrAnglePath=errorDistrAnglePath,
            errorDistrDistPath=errorDistrDistPath,
            useLidar=useLidar,
        )

    policy_kwargs = dict(features_extractor_class=SameExtractor)
    # model = PPO(
    #     ActorCriticCnnPolicy,
    #     env,
    #     policy_kwargs=policy_kwargs,
    #     verbose=1,
    #     tensorboard_log=f"{PATH}/logs/",
    #     n_steps=64,
    #     learning_rate=0.1,
    #     device="cuda",
    # )
    obs = env.reset()
    adv1 = Sb3_as_adv(env, load_dir=configs["model_location"])

    # Lists we want to save to panda dataframe
    brute_force_list = []
    rl_list = []
    traj_list = []
    len_traj_list = []
    len_traj_list_in_m = []
    rl_actions_to_crash_lists = []
    collided_obs_list = []

    # Info how often the exception hit wall needs to be handeled  # TODO find better handeling than discarding the run
    wall_discards = 0

    # for i in range(len)
    all_obst = deepcopy(env.env_real.obstacles)

    # Replay
    if load_location is not None:
        file = open(load_location, "rb")
        df = pkl.load(file)
        file.close()
        list_rl = []
        for prob_collision_with_Node in df["rl_prob"]:
            list_rl.append(calculate_collosion_order(prob_collision_with_Node))
        df["rl_final_prob"] = list_rl
        df["error"] = (df["rl_final_prob"] - df["brute_prob"]).abs()
        df = df.sort_values(by="error", ascending=False)

    run = 0
    while len(rl_actions_to_crash_lists) < amount_of_exploration:
        run = run + 1
        rospy.logdebug(f"[Crash and Remove] Running {run}. exploration")
        rospy.logdebug(
            f"[Crash and Remove] Runs saved {len(rl_actions_to_crash_lists)}"
        )
        # If we want to examine past experiences, replay will be enabled and multiple aspects of the code will be
        # retrieved from a dataframe instead of being generated. Additionally, a prompt will be included for the user
        # to confirm they have finished reviewing the output images

        env.env_real.map_ref = modify_map_out(
            env.env_real.map_ref,
            [],
            all_obst,
            color=(255, 255, 255),
            convert_back_to_grey=True,
        )
        env.env_real.map_ref_adv = deepcopy(env.env_real.map_ref)
        env.env_real.obstacles = deepcopy(all_obst)

        # Replay
        if replay_on:
            data_point = len(rl_actions_to_crash_lists)
            loaded_run_info = df.iloc[data_point]
            env.env_real.visualize = True
            _, traj_vanilla = env.env_real.reset(
                "adv1",
                start=[93, 122],
                goal=configs["goal"],
                new_traj=True,
                forced_traj=loaded_run_info["traj"],
            )
        # Generating based on existing trajectory
        elif initialTraj != None:
            _, traj_vanilla = env.env_real.reset(
                "adv1",
                start=[93, 122],
                goal=configs["goal"],
                new_traj=True,
                forced_traj=initialTraj,
            )
        # Generate a trajectory with a given start and endpoint
        elif start != None and goal != None:
            _, traj_vanilla = env.env_real.reset(
                "adv1", start=start, goal=goal, new_traj=True
            )
        # Randomly generate new trajectory
        else:
            _, traj_vanilla = env.env_real.reset("adv1", new_traj=True)
        traj_vanilla_org = deepcopy(traj_vanilla)
        prob_collision_brute = 0
        if use_brute_force_baseline:
            # Run the pruned brute force algorithm
            mcts_total_time = 0
            t4 = time.perf_counter()
            prob_collision_brute = run_pruned(env, adv1)
            mcts_total_time += time.perf_counter() - t4
            rospy.logdebug(
                f"[Crash and Remove] Brute force prob_collision is: {prob_collision_brute}"
            )
            rospy.logdebug(
                "[Crash and Remove] Brute force results in: {mcts_total_time} seconds"
            )

            # print("Press enter to start rl loop")
            if replay_on:
                rospy.logdebug(f"[Crash and Remove] Data_point is {data_point}")
                rospy.logdebug(
                    f"[Crash and Remove] \U0001F4C0 old error { loaded_run_info['error']}"
                )
                a = input()

        # --------------------------------- Run the RL loop --------------------------------------
        prob_collision_with_Node = []
        successful_actions = []
        collided_obstacles = []
        env.set_persisten_map(True)
        break_out = False
        temp_disable_replay = False

        for i in range(0, attempts):
            obs = env.reset()
            done = False
            step = 0
            actions = []
            while not done:
                # -- With replay --
                if replay_on and not temp_disable_replay:
                    if i < len(loaded_run_info["rl_actions_to_crash_lists"]):
                        if len(actions) < len(
                            loaded_run_info["rl_actions_to_crash_lists"][i]
                        ):
                            action = loaded_run_info["rl_actions_to_crash_lists"][i][
                                len(actions)
                            ]
                            action_was_loaded = True
                    if action_was_loaded:
                        action_was_loaded = False
                    else:
                        rospy.logdebug(
                            "[Crash and Remove] All crash paths have been displayed"
                        )
                        rospy.logdebug(
                            "[Crash and Remove] \U0001F4C0 type a to have the agent try to find new paths, other for next",
                        )
                        user_input = input()
                        if user_input == "a":
                            temp_disable_replay = True
                            action, _, _, _ = adv1.choose_action(obs)
                        else:
                            rospy.logdebug(
                                f"[Crash and Remove] Final rl estimate: {calculate_collosion_order(prob_collision_with_Node)}",
                            )
                            if use_brute_force_baseline:
                                rospy.logdebug(
                                    f"[Crash and Remove] Final brute estimate: {prob_collision_brute}"
                                )
                            break_out = True
                            break
                # -- With live-generated data --
                else:
                    action, _, _, _ = adv1.choose_action(obs)
                actions.append(action)

                obs, _, done, info = env.step(action)
                step = step + 1

                # ------ If collision happend ------
                if info["collision_status"]:
                    # going back expand length
                    forced_traj = traj_vanilla[
                        max(0, step - expand_length) : min(step + 1, len(traj_vanilla))
                    ]
                    if replay_on:
                        rospy.logdebug("[Crash and Remove] \U0001F4C0 collision_found")
                        a = input()

                    prob_collision_found = run_pruned(
                        env, adv1, forced_traj=forced_traj, reduce=True
                    )
                    env.env_real.trajectory_vanilla = deepcopy(traj_vanilla_org)
                    prob_collision_with_Node.append((prob_collision_found, step))
                    if replay_on:
                        rospy.logdebug(
                            f"[Crash and Remove] \U0001F4C0 crash prob {prob_collision_found}"
                        )
                        a = input()
                    else:
                        rospy.logdebug(
                            f"[Crash and Remove] Prob_collision_found crash prob {prob_collision_found}"
                        )

                    # TODO Move this out if we want to skip wall casts maybe do a do not save here
                    if "obstables_to_remove" in info:
                        obsts = info["obstables_to_remove"]
                        for obst in obsts:
                            collided_obstacles.append(obst[0])
                            env.env_real.map_ref = modify_map_out(
                                env.env_real.map_ref,
                                [obst[0]],
                                [],
                                color=(255, 255, 255),
                                convert_back_to_grey=True,
                            )
                            env.env_real.map_ref_adv = deepcopy(env.env_real.map_ref)
                            env.env_real.obstacles.pop(obst[1])
                        successful_actions.append(actions)
                    else:
                        rospy.logdebug(
                            "[Crash and Remove] There was a crash without an obstacle"
                        )
                        break_out = True
                        if replay_on:
                            rospy.logdebug(
                                "[Crash and Remove] \U0001F4C0 obstacles not remove"
                            )
                            a = input()
                    env.env_real.visu_adv_traj_map = deepcopy(env.env_real.map_ref_adv)
                    env.env_real.visu_adv_traj_map = (
                        env.env_real.visu_adv_traj_map.convert("RGB")
                    )

            # Replay
            if replay_on and temp_disable_replay:
                rospy.logdebug(
                    f"[Crash and Remove] \U0001F4C0 episode_end, type a to break out"
                )
                a = input()
                if a == "a":
                    break_out = True
            if break_out:
                if replay_on:
                    rl_actions_to_crash_lists.append((-1, -1))
                wall_discards = wall_discards + 1
                break

            rospy.logdebug(
                f"[Crash and Remove] Current_estimate:  {calculate_collosion_order(prob_collision_with_Node)}"
            )

        if not break_out:
            # TODO if len()<3 there might be problems
            rospy.loginfo(
                f"[Crash and Remove] Final RL estimate: {calculate_collosion_order(prob_collision_with_Node)}"
            )
            if use_brute_force_baseline:
                rospy.logdebug(
                    f"[Crash and Remove] Final brute estimate: {prob_collision_brute}"
                )
            if replay_on:
                a = input()
            if temp_disable_replay:
                rospy.logdebug(
                    f"[Crash and Remove] The old final RL estimate: {calculate_collosion_order(loaded_run_info['rl_prob'])}",
                )
                a = input()

            rl_list.append(prob_collision_with_Node)
            brute_force_list.append(prob_collision_brute)
            traj_list.append(traj_vanilla)
            len_traj_list.append(len(traj_vanilla))
            len_traj_list_in_m.append(traj_length(traj_vanilla))
            rl_actions_to_crash_lists.append(successful_actions)
            collided_obs_list.append(collided_obstacles)

            "every 500 runs we save the current state"
            if len(rl_actions_to_crash_lists) % 500 == 0 and not replay_on:
                data = {
                    "rl_prob": rl_list,
                    "brute_prob": brute_force_list,
                    "traj": traj_list,
                    "len_traj": len_traj_list,
                    "len_traj_list_in_m": len_traj_list_in_m,
                    "rl_actions_to_crash_lists": rl_actions_to_crash_lists,
                    "wall_discards": wall_discards,
                    "collided_obs": collided_obs_list,
                }
                df_currentRun = pd.DataFrame(data)
                df_oldRuns = pd.read_csv(save_location)
                df_save = pd.concat([df_currentRun, df_oldRuns])
                df_save.to_csv(save_location)

    # ----- Save the final results ----
    data = {
        "rl_prob": rl_list,
        "brute_prob": brute_force_list,
        "traj": traj_list,
        "len_traj": len_traj_list,
        "len_traj_list_in_m": len_traj_list_in_m,
        "rl_actions_to_crash_lists": rl_actions_to_crash_lists,
        "wall_discards": wall_discards,
        "collided_obs": collided_obs_list,
    }
    if not replay_on:
        df_currentRun = pd.DataFrame(data)
        try:
            df_oldRuns = pd.read_csv(save_location)
            pd.concat([df_oldRuns, df_currentRun], ignore_index=True).to_csv(
                save_location, index=False
            )
            # df_currentRun.concat(df_oldRuns, ignore_index=True).to_csv(
            #     save_location, index=False
            # )
        except:
            df_currentRun.to_csv(save_location, index=False)
        rospy.logdebug(f"[Crash and Remove] Wall_discards {wall_discards}")

    return data


if __name__ == "__main__":
    cfgs = configs[0]
    env_name = "robo_navigation-v01"
    use_brute_force_baseline = True

    runs = [
        # {'attempts': 5, 'expand_length': 2, 'amount_of_exploration': 200},
        # {'attempts': 10, 'expand_length': 3, 'amount_of_exploration': 200},
        # {'attempts': 25, 'expand_length': 5, 'amount_of_exploration': 200},
        {"attempts": 3, "expand_length": 2, "amount_of_exploration": 200},
    ]

    load_location = None
    load_settings = {"attempts": 10, "expand_length": 3, "amount_of_exploration": 300}
    ########################################################
    ## Uncomment "load_location" to replay from dataframe ##
    ########################################################
    load_location = f"risk_data_frames/bug_removed_1000/df_rl_v_brut_{load_settings['attempts']}_{load_settings['expand_length']}.obj"

    replay_on = bool(load_location)
    if replay_on:
        runs.insert(0, load_settings)

    for run in runs:
        attempts = run["attempts"]
        expand_length = run["expand_length"]
        amount_of_exploration = run["amount_of_exploration"]
        # save_location = f"{PATH}/risk_data_frames/bug_removed_{amount_of_exploration}/df_rl_v_brut_{attempts}_{expand_length}.obj"

        run_crash_and_remove(
            configs=cfgs,
            env_name=env_name,
            use_brute_force_baseline=use_brute_force_baseline,
            # save_location=save_location,
            replay_on=replay_on,
            # load_location=load_location,
            attempts=attempts,
            expand_length=expand_length,
            amount_of_exploration=amount_of_exploration,
        )
