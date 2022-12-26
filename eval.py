import numpy as np
import time

import config
from Environment import Environment
from ppo import Agent
from mcts import *
from SaveData import save_data, traj_length
from probability_cal import real_to_pixel , risk_calculation
from AutoLabel import read_data
from sklearn.svm import SVR

ACTION_SPACE_STEP_ADVERSARY = 6

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# choosing the action to take in pixel 
def choose_action(action_index):
    action_to_take = real_to_pixel(action_index)
    return action_to_take

# to take the probability of the action choosen
prob_occurance = [0.08,0.34,0.1,0.36,0.1]

def action_prob(action_index):
    return(prob_occurance[action_index])

def risk_cal(action_collision):
    return risk_calculation(action_collision)


def run_session_adv(config, test_mode, mcts_eval="IDA", ida_brute_combine=False):
    """
    this function starts the training or evaluation process of the simulation-gap-adversary
    be sure to pass the right path to your map in Environment.
    If visualization is wished (generate images of traj. while running) set visualized parameter in Environment = True
    @param config: config file with all the relevant environment variables and hyperparameters -> see config.py
    @param test_mode: test mode is true if we don't want to train a model but evaluate it (no exploration)
    @return: -
    """

    """ ML approach to find probability of collision"""

    data, _ = read_data("collision_data_000.csv")
    X_train, y_train = data[:,0:6], data[:,-1]
    regr = SVR()
    regr.fit(X_train, y_train)

    adv1 = Agent(n_actions=config['N_actions'], batch_size=config['batch_size'], gamma=config['gamma'],
                 alpha=config['lr'], n_epochs=config['n_epochs'],
                 model_arch=config['model_arch'], epsilon=config['epsilon'], entropy_coef=config['entropy_coef'], value_loss_coef=config['value_loss_coef'])

    print(config['start'], config['goal'])
    env = Environment(map_path=config['map_path'], relevant_segments=config['relevant_segments'],
                    done_after_collision=config['done_after_collision'], adversary=None,
                    visualize=True, start=config['start'], goal=config['goal'])

    if test_mode:
        adv1.load_models(path=config['log_name'] + '_models/', name='best')

    collision_data = []

    episode_counter = 1

    n_episodes = config['n_games']+1
    mcts_total_time = 0
    risk = 0
    cumm_po_ida = 0
    po_pc = 0
    if test_mode:
        # set the number of episodes to evaluate the model here
        n_episodes_eval = config['eval_ep']+1
        n_episodes = n_episodes_eval
    for episode in range(1, n_episodes):
        collision_ida = []
        reset_total_time = 0
        node_num = 1
        probs = []
        prob_collision = 0
        estimated_prob_collision = 0
        P_c_ida_ml = 0.000
        positions = []

        t0 = time.perf_counter()

        t2 = time.perf_counter()
        # every n_reset_nodes the map will sample new nodes for PRM. This parameter should be at least 5 times smaller than memory_size
        # the smaller this parameter is the better in general (less overfitting to current nodes) but it also extends training time considerably
        n_reset_nodes = 300
        if (episode % n_reset_nodes == 0) or episode == 1:
            observation, traj_vanilla = env.reset('adv1', new_nodes=True, start=config['start'], goal=config['goal'])
            obeservation_orig = observation
        else:
            observation, traj_vanilla = env.reset('adv1', start=config['start'], goal=config['goal'])
        reset_total_time += time.perf_counter()-t2

        step_counter = 0
        done = False
        p_o_ida = 1

        ep_states, ep_actions, ep_probs, ep_vals, ep_rewards, ep_entropy, ep_dones,  = [], [], [], [], [], [], []
        mcts = MonteCarloTreeSearch(config['N_actions'], traj_vanilla, env, adv1, obeservation_orig, test_mode= test_mode)
        action_space = mcts.expand()
        #print(action_space)

        pc_ida_max = 0
        
        if test_mode == True and mcts_eval == "BRUTE_FORCE"  or ida_brute_combine==True:
            t4 = time.perf_counter()
            observation_, reward, done, collision_status, _, prob_collision, risk_ep = mcts.take_action(action_space, done_after_collision= False)
            mcts_total_time += time.perf_counter() - t4
            length = traj_length(traj_vanilla)
            #temp_data = [traj_vanilla,traj_vanilla[0].coordinates[0],traj_vanilla[0].coordinates[1], traj_vanilla[len(traj_vanilla)-1].coordinates[0],traj_vanilla[len(traj_vanilla)-1].coordinates[1], len(traj_vanilla),length, prob_collision]
            #collision_data.append(temp_data)
            print("Risk: ", np.sqrt(risk_ep))
            risk += risk_ep
            
        done = False
        #collision_status = False
        env.reset_traj()
        while not done:
            action_index, prob, val, raw_probs = adv1.choose_action(observation, test_mode=test_mode)

            if test_mode:
                if mcts_eval == "BRUTE_FORCE":
                    done = True

                if mcts_eval=="IDA" or ida_brute_combine==True:

                    action_index, prob, val, raw_probs = adv1.choose_action(observation, test_mode=test_mode)
                    pos_offset = choose_action(action_index)
                    action_prob_value = action_prob(action_index)
                    action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(config['N_actions']/2)*ACTION_SPACE_STEP_ADVERSARY))
                    observation_, reward, done, collision_status, _, position_old = env.step_adv1(action_angle_offset,action_prob_value)
                    probs_all = np.round(raw_probs.cpu().detach().numpy().squeeze(0),4)
                    #print(probs_all)
                    traj_temp = traj_vanilla
                    if(len(traj_vanilla) > 2 and step_counter > 0):
                        traj_temp = traj_vanilla[step_counter:len(traj_vanilla)]

                    length = traj_length(traj_temp)
                    my_data = [[traj_temp[0].coordinates[0],traj_temp[0].coordinates[1], traj_temp[len(traj_temp)-1].coordinates[0],traj_temp[len(traj_temp)-1].coordinates[1], len(traj_temp),length]]
                    P_c_ida_ml = abs(regr.predict(my_data)[0])
                    if P_c_ida_ml > pc_ida_max:
                        pc_ida_max = P_c_ida_ml
                    probs.append(probs_all)
                    positions.append(position_old)
                    p_o_ida *= action_prob_value

            # if collision_status:
            #     print("Untried actions: ", mcts.untried_actions())
            step_counter += 1

            if done:
                if collision_status == 1:
                    collision_ida.append(1)
                    print("\U0000274c",bcolors.BOLD + bcolors.FAIL + "Collision occured and not reached" + bcolors.ENDC)
                    
                else:
                    collision_ida.append(0)
                    print("\U00002705",bcolors.BOLD + bcolors.OKGREEN + "Reached at the destination" + bcolors.ENDC)   

                if test_mode:
                    if mcts_eval=="IDA" or ida_brute_combine:
                        print("p_o_ida: ", p_o_ida)
                        cumm_po_ida += p_o_ida

                        new_action_index, prob_prev , node_num, pos_prev = mcts.find_best_child(probs, positions, node_num=1)
                        while (len(traj_vanilla) - node_num-1)>0:
                            done = False
                            env.reset_traj(node_num, pos=pos_prev)
                            prev_node_num = node_num
                            probs = []
                            node_num+=1
                            positions = []
                            
                            """ swapnil"""
                            pos_offset = choose_action(new_action_index)
                            action_prob_value = action_prob(new_action_index)
                            #action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(config['N_actions']/2)*ACTION_SPACE_STEP_ADVERSARY))
                            #observation_, reward, done, collision_status, _, position_old = env.step_adv1(pos_offset,action_prob_value)

                            """ Previous"""
                            action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * new_action_index - (int(config['N_actions']/2)*ACTION_SPACE_STEP_ADVERSARY))
                            observation_, reward, done, collision_status, _, position_old = env.step_adv1(action_angle_offset, action_prob_value)
                            
                            observation = observation_
                            if done:
                                node_num = len(traj_vanilla)
                                if collision_status == 1:
                                    collision_ida.append(1)
                                    print("\U0000274c",bcolors.BOLD + bcolors.WARNING + "Collision occured and not reached" + bcolors.ENDC)
                                else:
                                    collision_ida.append(0)
                                    print("\U00002705",bcolors.BOLD + bcolors.OKBLUE + "Reached at the destination" + bcolors.ENDC)
                            while not done:
                                action_index, prob, val, raw_probs = adv1.choose_action(observation, test_mode=test_mode)
                                
                                pos_offset = choose_action(action_index)
                                action_prob_value = action_prob(action_index)
                                #action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(config['N_actions']/2)*ACTION_SPACE_STEP_ADVERSARY))
                                #observation_, reward, done, collision_status, _, position_old = env.step_adv1(pos_offset,action_prob_value)

                                """ Previous"""
                                action_angle_offset = np.deg2rad(ACTION_SPACE_STEP_ADVERSARY * action_index - (int(config['N_actions']/2)*ACTION_SPACE_STEP_ADVERSARY))
                                observation_, reward, done, collision_status, _, position_old = env.step_adv1(action_angle_offset,action_prob_value)
                                
                                probs_all = np.round(raw_probs.cpu().detach().numpy().squeeze(0),4)
                                probs.append(probs_all)
                                positions.append(position_old)

                                if done:
                                    new_action_index, prob_prev , node_num, pos_prev = mcts.find_best_child(probs, positions, node_num, pos_prev=pos_prev)
                                    if node_num == prev_node_num:
                                        node_num += 1
                                        pos_prev = [-1,-1]
                                        new_action_index = 2
                                    if collision_status == 1:
                                        collision_ida.append(1)
                                        print("\U0000274c",bcolors.BOLD + bcolors.WARNING + "Collision occured and not reached" + bcolors.ENDC)
                                    else:
                                        collision_ida.append(0)
                                        print("\U00002705",bcolors.BOLD + bcolors.OKBLUE + "Reached at the destination" + bcolors.ENDC)
                                        #break
                                observation = observation_
                            
          
                adv1.remember(ep_states, ep_actions, ep_probs, ep_vals, ep_rewards, ep_entropy, ep_dones)
            observation = observation_

        """ Probability of collision for IDA and collect data for logging""" 

        if mcts_eval == "IDA" or ida_brute_combine:
            constant = 0.01
            learning_rate = 0.1
            total_success = sum(x == 0 for x in collision_ida)
            total_collisions = sum(x == 1 for x in collision_ida)
            residual = (((total_collisions-total_success + constant)/(total_success+total_collisions+constant))* (1/(len(traj_vanilla)-step_counter+constant)) * learning_rate)
            estimated_prob_collision = round(abs(pc_ida_max + residual),4)
            po_pc += estimated_prob_collision * p_o_ida
            p_o_ida = 1
            length = traj_length(traj_vanilla)

        temp_data = [traj_vanilla,
                    traj_vanilla[0].coordinates[0],traj_vanilla[0].coordinates[1], 
                    traj_vanilla[len(traj_vanilla)-1].coordinates[0],
                    traj_vanilla[len(traj_vanilla)-1].coordinates[1], 
                    len(traj_vanilla),
                    length,
                    prob_collision,
                    estimated_prob_collision,
                    round(P_c_ida_ml,4)]
        
        collision_data.append(temp_data)
        
        episode_counter += 1
        print("episode_counter: ", episode_counter)
    save_data(data=collision_data, create_header=True)

    if mcts_eval == "BRUTE_FORCE" or ida_brute_combine:
        cumm_risk = risk/(n_episodes-1)
        
    else:
        cumm_risk = po_pc/(n_episodes-1)

    print("Risk: ", cumm_risk)
    print("Risk sqrt: ", np.sqrt(cumm_risk))

    return np.sqrt(cumm_risk)

def mains(mode=True, mcts_eval="IDA", combined_eval=False):
    n_sessions = 1
    done = False
    """ mcts_eval: BRUTE_FORCE, BINARY_SEARCH, IDA"""
    for i in range(0, n_sessions):
        risk = run_session_adv(config.configs[i], test_mode=mode, mcts_eval=mcts_eval, ida_brute_combine=combined_eval)

    print('evaluation finished')
    done = True

    return done, risk


if __name__ == '__main__':
    mains(mode=True)
