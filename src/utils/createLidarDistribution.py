import csv
import numpy as np


def createEntries(start, end, count, isPositive):
    for _ in range(count):
        randomValueX = np.random.randint(start, end) * 0.001
        randomValueY = np.random.randint(start, end) * 0.001
        if isPositive:
            dist = np.sqrt(np.square(randomValueX) + np.square(randomValueY))
        else:
            dist = (-1) * np.sqrt(np.square(randomValueX) + np.square(randomValueY))
        with open(
            "/home/ros/catkin_ws/src/robotino/logs/error_dist_csvs/lidar.csv",
            "a",
        ) as f1:
            write = csv.writer(f1)
            write.writerow([dist])


if __name__ == "__main__":
    createEntries(0, 8, 28, True)
    createEntries(8, 16, 24, True)
    createEntries(16, 24, 19, True)
    createEntries(24, 32, 17, True)
    createEntries(32, 40, 7, True)
    createEntries(40, 48, 4, True)
    createEntries(0, 8, 28, False)
    createEntries(8, 16, 24, False)
    createEntries(16, 24, 19, False)
    createEntries(24, 32, 17, False)
    createEntries(32, 40, 7, False)
    createEntries(40, 48, 4, False)
