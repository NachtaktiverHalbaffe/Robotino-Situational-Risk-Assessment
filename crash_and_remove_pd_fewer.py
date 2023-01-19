import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np
from crash_and_remove import calculate_collsion_order



name1 = "risk_data_frames/1000/df_rl_v_brut_traj_3_10.obj"
name2 = "risk_data_frames/new_test_1000/df_rl_v_brut_10_3.obj"
name3 = "risk_data_frames/100/df_rl_v_brut_traj_3_25.obj"
name4 = "risk_data_frames/1000/df_rl_v_brut_traj_5_25.obj"

file = open(name1,'rb')
df = pkl.load(file)
file.close()

"available keys"
# 'rl_prob':rl_list,
# 'brute_prob':brute_force_list,
# 'traj':traj_list,
# 'len_traj':traj_list,
# 'len_traj_list_in_m':len_traj_list_in_m,
# 'rl_actions_to_crash_lists':rl_actions_to_crash_lists})

list_rl = []
for prob_collision_with_Node in df['rl_prob']:
    list_rl.append(calculate_collsion_order(prob_collision_with_Node))
df['rl_prob'] = list_rl

count, division = np.histogram((df['rl_prob']-df['brute_prob']).abs(), bins=12)

# Create the first subplot
plt.subplot(1, 2, 1)
plt.subplot(1, 2, 1).set_title("Basic,old system, 1000runs")

ax = (df['rl_prob']-df['brute_prob']).abs().plot.hist(bins=division, alpha=0.5, label = 'error')

shorter_than_3 = []
for i, traj in enumerate(df['traj']):
    if len(traj)<4:
        shorter_than_3.append(i)
print(len(shorter_than_3))

df = df.drop(index=shorter_than_3)

ax = (df['rl_prob']-df['brute_prob']).abs().plot.hist(bins=division, alpha=0.5,label = 'min len 4')

# Create the second subplot
df_sliced = df.drop(df[(df['rl_prob']-df['brute_prob'])<=0].index)
ax = (df_sliced['rl_prob']-df_sliced['brute_prob']).abs().plot.hist(bins=division, alpha=0.5,label = 'rl is higher')













file = open(name2,'rb')
df = pkl.load(file)
file.close()
list_rl = []


for prob_collision_with_Node in df['rl_prob']:
    list_rl.append(calculate_collsion_order(prob_collision_with_Node))
df['rl_prob'] = list_rl

# count, division = np.histogram((df['rl_prob']-df['brute_prob']).abs(), bins=12)
plt.legend()

# Create the first subplot
plt.subplot(1, 2, 2)
plt.subplot(1, 2, 2).set_title("Basic, removed Error that was reducing traj, 1000runs")

ax = (df['rl_prob']-df['brute_prob']).abs().plot.hist(bins=division, alpha=0.5, label = 'error')

shorter_than_3 = []
for i, traj in enumerate(df['traj']):
    if len(traj)<4:
        shorter_than_3.append(i)
print(len(shorter_than_3))

df = df.drop(index=shorter_than_3)

ax = (df['rl_prob']-df['brute_prob']).abs().plot.hist(bins=division, alpha=0.5,label = 'min len 4')

# Create the second subplot
df_sliced = df.drop(df[(df['rl_prob']-df['brute_prob'])<=0].index)
ax = (df_sliced['rl_prob']-df_sliced['brute_prob']).abs().plot.hist(bins=division, alpha=0.5,label = 'rl is higher')
plt.legend()

plt.show()