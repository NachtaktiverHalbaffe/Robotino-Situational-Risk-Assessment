import csv
import numpy as np


def traj_length(traj_vanilla):
    """
    Calculates the length of an trajectory

    Args:
        traj_vanilla (list(Node)): The trajectory which length should be calculated

    Returns:
        length (float): Length of the trajectory. Is rounded to 4 decimal places
    """
    prev = traj_vanilla[0].coordinates
    length = 0

    for i in range(1, len(traj_vanilla)):
        # Coordinate of current node
        curr = traj_vanilla[i].coordinates
        # Simple vector calculation: linear distance between prev-node and curr-node
        length += np.linalg.norm(np.array(curr) - np.array(prev))
        # Set current node as last node
        prev = curr

    length = np.round(length, 4)
    return length


def save_data(data=None, create_header=True):
    """
    Write data to a csv file. CSV contains  columns "Trajectory",  "StartX", "StartY",  "EndX", "EndY",\
    "N_nodes", "length", "Prob_collision_Brute_force", "Prob_collision_IDA" and "Expected Probability Collision".

    Args:
        data (list, optional): Data to write.  Defaults to None
        create_header (bool, optional): If a header should be also written in csv. Defaults to True
    """
    if create_header:
        header = [
            "Trajectory",
            "StartX",
            "StartY",
            "EndX",
            "EndY",
            "N_nodes",
            "length",
            "Prob_collision_Brute_force",
            "Prob_collision_IDA",
            "Expected Probability Collision",
        ]
    # data = []
    # data2 = [[10,10], [20,20], 5, 0.75]
    # data1 = [[10,10], [20,20], 5, 0.75]
    # data.append(data1)
    # data.append(data2)

    with open("collision_data.csv", "w", newline="") as f:
        writer = csv.writer(f)

        # write the header
        if create_header:
            writer.writerow(header)

        # write the data
        for d in data:
            writer.writerow(d)


# ------------ Testing/Debugging -----------------
if __name__ == "__main__":
    # save_data()
    curr = [52, 74]
    prev = [49, 83]
    length = np.linalg.norm(np.array(curr) - np.array(prev))
    print(length)
