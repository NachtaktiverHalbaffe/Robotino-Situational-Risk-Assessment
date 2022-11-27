import csv
import numpy as np

def traj_length(traj_vanilla):
    prev = traj_vanilla[0].coordinates
    length = 0
    for i in range(1, len(traj_vanilla)):
        curr = traj_vanilla[i].coordinates
        length += np.linalg.norm(np.array(curr) - np.array(prev))
        prev = curr
    return length

def save_data(data=None, create_header=True):
    if create_header:
        header = ['StartX','StartY', 'EndX','EndY', 'N_nodes', 'length', 'Prob_collision']
    # data = []
    # data2 = [[10,10], [20,20], 5, 0.75]
    # data1 = [[10,10], [20,20], 5, 0.75]
    # data.append(data1)
    # data.append(data2)


    with open('collision_data.csv', 'w', newline='') as f:
        writer = csv.writer(f)

        # write the header
        if create_header:
            writer.writerow(header)

        # write the data
        for d in data:
            writer.writerow(d)
    
if __name__ == '__main__':
    save_data()