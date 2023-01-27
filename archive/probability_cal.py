import numpy as np
import copy


def real_to_pixel(action_index):
    """
    Converts real value into the pixel space of the image

    Args:
        action_index (int): Index of the taken action from the adversary

    Returns:
        action_to_take (int): converted value
    """
    x_max = 200
    y_max = 200
    x_room = 400
    y_room = 400
    pixel_value = (x_room * y_room) / (x_max * y_max)
    action_to_take = (action_index - 2) * 6

    return action_to_take


def action_prob_cal(action_index):
    """

    Args:
        action_index (int): Index of the action in the action space to take

    Returns:
        float: Probability of the action
    """
    if action_index == 0:
        return 0.08
    elif action_index == 1:
        return 0.34
    elif action_index == 2:
        return 0.1
    elif action_index == 3:
        return 0.36
    elif action_index == 4:
        return 0.1


def risk_calculation(action_index):
    """
    Calculates the risk when taking the action

    Args:
        action_index (list(int)): Actions taken

    Returns:
        final_probability (float): Probability of the taken actions. Calculation (in LaTeX like notation):\
                                                prod_{item\in res} (probability of a action ^how oft the action was taken)\
                                                 * len(action_index)/prod_{item\in probability_1} action_index.count(items)! 
    """
    action_list = [-4, -2, 0, 2, 4]
    real_probability = [0.06, 0.26, 0.1, 0.3, 0.06]
    probaility_1 = []

    for items in action_list:
        # Count how often a action in the action list was taken
        res = action_index.count(items)
        probaility_1.append(res)

    fact = 1
    for items in probaility_1:
        # fact=prod_{item\in probability_1} action_index.count(items)! (LaTeX notation)
        fact = fact * np.math.factorial(items)
    action_len = len(action_index)
    # advance_func =  action_len!/ fact = len(action_index)/prod_{item\in probability_1} action_index.count(items)! (LaTeX notation)  (LaTeX notation)
    advance_func = np.math.factorial(action_len) / fact

    # res= real_probability^probability_1 = probability of a action ^how oft the action was taken
    res = [pow(a, b) for a, b in zip(real_probability, probaility_1)]
    result = 1
    for items in res:
        # result= prod_{item\in res} (probability of a action ^how oft the action was taken) (LaTeX notation)
        result = result * items

    final_probability = result * advance_func
    return final_probability


def cumm_risk(action_taken):
    """
    Args:
        action_index (list(int)): Index of the taken actions in the action space

    Returns:
        cummulative_prob (float): The summed probability of all taken action\
        which ar within the action_space [0, 2, 4]
    """
    i = 0
    cummulative_prob = 0
    action_taken1 = copy.deepcopy(action_taken)
    cummulative_prob += risk_calculation(action_taken1)

    while i < len(action_taken1):
        # Remap action space from [-4, -2, 0, 2, 4, 6] to [-4, -2, 2, 4, 6]
        #  => 0 ist mapped to +-2 and all other positive values are mapped one step above
        if action_taken1[i] == 0:
            action_taken1[i] = -2
            cummulative_prob += risk_calculation(action_taken1)
            action_taken1[i] = 2
            cummulative_prob += risk_calculation(action_taken1)
        elif action_taken1[i] == 2:
            action_taken1[i] = 4
            cummulative_prob += risk_calculation(action_taken1)
        elif action_taken1[i] == 4:
            action_taken1[i] = 6
            cummulative_prob += risk_calculation(action_taken1)
        # Only cummulative_prob calculated for (original) action space [0, 2, 4]
        elif action_taken1[i] == 6:
            i = i + 1
        elif action_taken1[i] == -2:
            i = i + 1
        elif action_taken1[i] == -4:
            i = i + 1
    return cummulative_prob
