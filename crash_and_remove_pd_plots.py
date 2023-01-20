import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np
from crash_and_remove import calculate_collsion_order

#########################
####Choose Files here####
#########################
sub_folder = "bug_removed_1000"
"here the indivdual runs that should be displayed are needed, write the attemps and the expansion size, first should we worst if you use the same bins"
same_bins = True
params = [(10,3),
          (10,5),
          (25,3),
          (25,5),
          ]

# sub_folder = "bug_removed_200"
# params = [(3,2),
#           (5,2),
#           (10,3),
#           (25,5),
#           ]


#############################
####The rest is automated####
#############################

filenames = ["./risk_data_frames/"+sub_folder+"/df_rl_v_brut_{}_{}.obj".format(a, b) for a, b in params]

plot_names = ["RL attemps:{}    Expansions size:{}".format(a, b) for a, b in params]

for i, name in enumerate(filenames):
    file = open(name,'rb')
    df = pkl.load(file)
    file.close()

    list_rl = []
    for prob_collision_with_Node in df['rl_prob']:
        list_rl.append(calculate_collsion_order(prob_collision_with_Node))
    df['rl_prob'] = list_rl

    if i==0 or not same_bins:
        count, division = np.histogram((df['rl_prob']-df['brute_prob']).abs(), bins=12)

    # Create the subplot
    plt.subplot(2, 2, i + 1)
    mean= (df['rl_prob']-df['brute_prob']).abs().mean()
    plt.subplot(2, 2, i + 1).set_title(plot_names[i]+f'  Mean: {round(mean,2)}')

    ax = (df['rl_prob']-df['brute_prob']).abs().plot.hist(bins=division, alpha=0.5, label = 'error')

    shorter_than = []
    for j, traj in enumerate(df['traj']):
        if len(traj)<=params[i][1]:
            shorter_than.append(j)
    print(len(shorter_than))

    # df = df.drop(index=shorter_than)

    ax = (df['rl_prob']-df['brute_prob']).abs().plot.hist(bins=division, alpha=0.5,label = f'min len {params[i][1]+1}')

    df_sliced = df.drop(df[(df['rl_prob']-df['brute_prob'])<=0].index)
    ax = (df_sliced['rl_prob']-df_sliced['brute_prob']).abs().plot.hist(bins=division, alpha=0.5,label = 'rl is higher')

    plt.legend()
plt.show()