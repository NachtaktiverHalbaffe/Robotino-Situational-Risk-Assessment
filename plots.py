import time
import numpy as np
import evaluate_logs

from CSV_Logger import CSV_Reader
avg_window = 1000   # determines how many episodes are averaged in plots

# log_reader = CSV_Reader('prot_1', test_mode=False)
# evaluate_logs.plot_collisions('prot_1', log_reader, avg_window=avg_window)
# evaluate_logs.plot_length_ratios('prot_1', log_reader, avg_window=avg_window)
# evaluate_logs.plot_scores('prot_1', log_reader, avg_window=avg_window)
print('probs adv:')
#evaluate_logs.calc_probabilities(log_reader)  # for evaluation: calculates and prints the frequencies of chosen actions (for histogram)

log_reader = CSV_Reader('adv_1', test_mode=False)
evaluate_logs.plot_rewards('adv_1', log_reader, avg_window=3*avg_window)    # rewards (for adversary) are per step instead of per episode -> choose bigger average window
evaluate_logs.plot_scores('adv_1', log_reader, avg_window=avg_window)
evaluate_logs.plot_collisions('adv_1', log_reader, avg_window=avg_window)
print('probs adv:')
evaluate_logs.calc_probabilities(log_reader)  # for evaluation: calculates and prints the frequencies of chosen actions (for histogram)
