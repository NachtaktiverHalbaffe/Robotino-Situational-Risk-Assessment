from sre_constants import SUCCESS
import numpy as np
import torch
import time

from prettytable import PrettyTable
from torch.distributions import Categorical

import config
from Environment import Environment
from ppo import Agent
from archive.CSV_Logger import CSV_Logger

ACTION_SPACE_STEP_ADVERSARY = 5
ACTION_SPACE_STEP_PROT = 4  # has to be even!


def count_parameters(model):
    """
    Counts the number of parameters in the neural network

    Args:
        model (Agent): Neural network model we want to count the parameters of

    Returns:
        total_params (int): Amount of total trainable parameters
    """
    table = PrettyTable(["Modules", "Parameters"])
    total_params = 0
    # Counting
    for name, parameter in model.named_parameters():
        if not parameter.requires_grad:
            continue
        param = parameter.numel()
        # Add name of model and number of parameters as row to table
        table.add_row([name, param])
        total_params += param

    # Logging
    print(table)
    print("Total Trainable Params:", total_params)
    return total_params


def run_session_prot(config, test_mode):
    """
    This function starts the training or evaluation process of the protagonist be\
    sure to pass the right path to your map in Environment.

    If visualization is wished (generate images of traj. while running) set visualized parameter in Environment = True

    Args:
        config: config file with all the relevant environment variables and hyperparameters -> see config.py
        test_mode (bool): test mode is True if we don't want to train a model but evaluate it (no exploration)
    """
    ######################################################################################################
    # ----------------------------------------------------------  Setup --------------------------------------------------------------
    ######################################################################################################
    if test_mode:
        # torch.manual_seed(0)
        # np.random.seed(0)
        pass

    adv1 = Agent(
        n_actions=5,
        batch_size=config["batch_size"],
        gamma=config["gamma"],
        alpha=config["lr"],
        n_epochs=config["n_epochs"],
        model_arch=config["model_arch"],
        epsilon=config["epsilon"],
        entropy_coef=config["entropy_coef"],
        value_loss_coef=config["value_loss_coef"],
    )

    adv1.load_models(path="adv_1" + "_models/", name="best")

    prot = Agent(
        n_actions=5,
        batch_size=config["batch_size"],
        gamma=config["gamma"],
        alpha=config["lr"],
        n_epochs=config["n_epochs"],
        model_arch=config["model_arch"],
        epsilon=config["epsilon"],
        entropy_coef=config["entropy_coef"],
        value_loss_coef=config["value_loss_coef"],
    )

    env = Environment(
        map_path=config["map_path"],
        relevant_segments=config["relevant_segments"],
        done_after_collision=config["done_after_collision"],
        adversary=adv1,
        visualize=True,
    )

    # count controllable parameters
    count_parameters(prot.actor)

    if test_mode:
        prot.load_models(path=config["log_name"] + "_models/", name="best")

    logger = CSV_Logger(
        N_epochs=config["n_epochs"],
        N_games=config["n_games"],
        memory_size=config["memory_size"],
        batch_size=config["batch_size"],
        gamma=config["gamma"],
        lr=config["lr"],
        anneal_lr=config["anneal_lr"],
        epsilon=config["epsilon"],
        entropy_coef=config["entropy_coef"],
        value_loss_coef=config["value_loss_coef"],
        relevant_segments=config["relevant_segments"],
        done_after_collision=config["done_after_collision"],
        model_arch=config["model_arch"],
        test_mode=test_mode,
        path=config["log_name"],
    )

    # Reinforcement learning performance variables
    avg_score_best = -1
    score_history = []
    reward_history = []

    # training progress variables
    learn_iters = 0
    n_steps = 0
    episode_counter = 1
    n_learn_iters = int(config["n_games"] / config["memory_size"])

    steps_episodes_log, steps_probs_log, steps_val_log, steps_reward_log = (
        [],
        [],
        [],
        [],
    )
    (
        ep_episodes_log,
        ep_probs_log,
        ep_val_log,
        ep_reward_log,
        ep_collisions_log,
        ep_length_ratio_log,
    ) = ([], [], [], [], [], [])
    collision_status_list = []

    n_episodes = config["n_games"] + 1
    if test_mode:
        # set the number of episodes to evaluate the model here
        # TODO Specify this via constant
        n_episodes_eval = 300
        n_episodes = n_episodes_eval

    ######################################################################################################################
    # ----------------------------------------------------------------- Run the session -----------------------------------------------------------------
    ######################################################################################################################
    for episode in range(1, n_episodes):
        # Init params
        choose_action_total_time = 0
        reset_total_time = 0
        step_total_time = 0

        t0 = time.perf_counter()
        t2 = time.perf_counter()
        # every n_reset_nodes the map will sample new nodes for PRM. This parameter should be at least 5 times smaller than memory_size
        # the smaller this parameter is the better in general (less overfitting to current nodes) but it also extends training time considerably
        n_reset_nodes = 300
        if (episode % n_reset_nodes == 0) or episode == 1:
            # Has run n_reset_nodes => sample new graph for PRM and new trajectory
            observation = env.reset(
                "prot", new_nodes=True, start=config["start"], goal=config["goal"]
            )
        else:
            # Hasn't run n_reset_nodes => just sample new trajectory
            observation = env.reset("prot", start=config["start"], goal=config["goal"])
        # Get stop time of resetting process
        reset_total_time += time.perf_counter() - t2

        step_counter = 0

        done = False
        score = 0

        ############################################
        #### ------------- Run one episode -----------------
        ############################################
        ep_states, ep_actions, ep_probs, ep_vals, ep_rewards, ep_entropy, ep_dones = (
            [],
            [],
            [],
            [],
            [],
            [],
            [],
        )
        while not done:
            ##############################
            # --------- Run one step -----------
            ##############################
            step_counter += 1

            ##############################
            # Choose and perform one action
            # in simulation environment
            ##############################
            t1 = time.perf_counter()
            # Choose action
            action_index, prob, val, raw_probs = prot.choose_action(
                observation, test_mode=test_mode
            )
            choose_action_total_time += time.perf_counter() - t1
            t3 = time.perf_counter()
            # Perform step
            observation_, reward, done, info, _ = env.step_prot(
                ACTION_SPACE_STEP_PROT * action_index - 1
            )
            step_total_time += time.perf_counter() - t3
            collision_status = info[0]
            length_ratio = info[1]

            # Append necessary step data to track session
            steps_episodes_log.append(episode_counter)
            steps_probs_log.append(
                np.round(raw_probs.cpu().detach().numpy().squeeze(0), 2)
            )
            steps_val_log.append(np.round(val, 2))
            steps_reward_log.append(np.round(reward, 2))

            # Calculate performance data of episode
            n_steps += 1
            score += reward
            reward_history.append(reward)

            # Append necessary episode data to track episode
            ep_states.append(observation)
            ep_actions.append(action_index)
            ep_probs.append(prob)
            ep_vals.append(val)
            ep_rewards.append(reward)
            ep_entropy.append(
                torch.squeeze(Categorical(raw_probs).entropy().detach(), 0).item()
            )
            ep_dones.append(done)

            if done:
                ############################
                # Episode will terminate
                ############################
                # Write protagonist parameters into memory
                prot.remember(
                    ep_states,
                    ep_actions,
                    ep_probs,
                    ep_vals,
                    ep_rewards,
                    ep_entropy,
                    ep_dones,
                )
                collision_status_list.append(collision_status)

            observation = observation_

        if episode % config["memory_size"] == 0:
            if config["anneal_lr"]:
                frac = 1.0 - (learn_iters - 1.0) / n_learn_iters
                lrnow = frac * config["lr"]
                prot.actor.optimizer.param_groups[0]["lr"] = lrnow
                prot.critic.optimizer.param_groups[0]["lr"] = lrnow
                print("annealing_lr:", lrnow)

            prot.learn(test_mode=test_mode)
            learn_iters += 1

        ################################
        # Evaluation and logging of episode
        ################################
        episode_counter += 1
        ep_episodes_log.append(steps_episodes_log)
        ep_probs_log.append(steps_probs_log)
        ep_val_log.append(steps_val_log)
        ep_reward_log.append(steps_reward_log)
        ep_length_ratio_log.append(np.round(length_ratio, 2))
        ep_collisions_log.append(np.round(collision_status, 2))

        # Log data into CSV if specified log-interval is reached
        if (episode_counter - 1) % config["log_interval"] == 0:
            logger.add_rows(
                episodes=ep_episodes_log,
                probs=ep_probs_log,
                values=ep_val_log,
                rewards=ep_reward_log,
                collision_status=ep_collisions_log,
                length_ratio=ep_length_ratio_log,
            )
            # Reset episode logging variables
            (
                ep_episodes_log,
                ep_probs_log,
                ep_val_log,
                ep_reward_log,
                ep_collisions_log,
                ep_length_ratio_log,
            ) = ([], [], [], [], [], [])

        steps_episodes_log, steps_probs_log, steps_val_log, steps_reward_log = (
            [],
            [],
            [],
            [],
        )

        #  Average sliding window
        score_history.append(score)
        # the avg_window variables are for prints while training and determine the amount of episodes/steps we want to average over
        avg_window = 4000
        avg_window_score = 2000

        avg_score = np.mean(score_history[-avg_window:])
        avg_collision = np.mean(collision_status_list[-avg_window_score:])
        #  Apply average sliding windows if history is long enough
        if len(reward_history) > avg_window:
            reward_history = reward_history[-avg_window:]
        if len(score_history) > avg_window_score:
            score_history = score_history[-avg_window_score:]
            collision_status_list = collision_status_list[-avg_window_score:]
        ####################################################################
        # ---------------------- End of session  ==> Save data -----------------------------
        ####################################################################
        if (episode % 800 == 0) and (not test_mode):
            # Save weights of every 800th episode
            prot.save_models(path=config["log_name"] + "_models/", name=str(episode))
        # Save model if episode had the best score ==> episode was the best
        if (
            (avg_score > avg_score_best + 0.02)
            and (not test_mode)
            and (episode > avg_window_score)
        ):
            print(">>>>> new best !!! <<<<<")
            avg_score_best = avg_score
            prot.save_models(path=config["log_name"] + "_models/", name="best")

        print(
            "episode",
            episode,
            "score %.1f" % score,
            "avg score %.2f" % avg_score,
            "avg collision %.2f" % avg_collision,
            "time_steps",
            n_steps,
            "learning_steps",
            learn_iters,
        )
        print("episode_time:", time.perf_counter() - t0)
