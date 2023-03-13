import rostopic
import rospy
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def plotErrorDist(
    title: str, xLabel: str, savePath: str, segments: list, probabilities: list
):
    """
    Plots the error distribution into a barchart and saves it as an PNG-file

    Args:
        title (str): The title (label) of the diagramm
        xLabel (str): Label of the x-axis
        segments (list): The segment values from the distribution
        probabilities (list): The probability values from the distribution
    """
    plt.bar(segments, probabilities, width=0.5)
    plt.xlabel(xLabel)
    plt.ylabel("Wahrscheinlichkeit")
    plt.title(title)
    plt.savefig(savePath)
    plt.close()
    plt.cla()
    plt.clf()


def loadErrorValues(path: str, bins: int = 5, precision: int = 8, useUpperBins=False):
    """
    Loads error values from a CSV file and creates a histogram from them

    Args:
        path (str): The path to the CSV file
        bins (int, optional): Number of segments into which the error distribution should be divided
        precision (int, optional): The number of decimal places to which the probabilities are rounded

    Returns:
        segments (list of float): The segments from the error distribution
        probabilities (list of float): The corresponding probability of each segment
    """
    file = Path(str(path).strip().replace("'", ""))
    if file.exists():
        data = pd.read_csv(path, header=None)
        probabilities, segments = np.histogram(data, bins=bins, density=False)
        if len(segments) != bins:
            if useUpperBins:
                segments = segments[1:]
            else:
                segments = segments[: len(segments) - 1]
        probabilities = np.divide(probabilities, len(data))
        probabilities = np.round(probabilities, precision)

        return segments, probabilities
    else:
        raise FileExistsError("Error distribution doesn't exist")


def generateErrorDistributionAngle(path: str, precision: int = 8, bins=5):
    """
    Generate a error distirbution for the angle metric with error values from a CSV File. Because the values are already including negative values,\
    it only loads the histogram and does logging 

    Args:
        path (str): The path to the CSV file
        precision (int, optional): The number of decimal places to which the probabilities are rounded

    Returns:
        segments (list of float): The segments from the error distribution
        probabilities (list of float): The corresponding probability of each segment
    """
    segments, probabilities = loadErrorValues(path, bins=bins, precision=precision)

    try:
        rostopic.get_topic_class("/rosout")
        rospy.logdebug(
            f"[Monitored Space Observer] Loaded error distribution from {path}.\nSegments: {segments}\nProbabilities of segments: {probabilities}"
        )
    except:
        print(
            f"[Monitored Space Observer] Loaded error distribution from {path}.\nSegments: {segments}\nProbabilities of segments: {probabilities}"
        )

    return segments, probabilities


def generateErrorDistributionDistance(path: str, precision: int = 8, bins=5):
    """
    Generate a error distirbution for the distance metric with error values from a CSV File. Distance metric only saves positive values\
    (to the nature of euclidian length calculation), so its assumend that the distirbution is symmetrical to 0 ==> Mirror list and interpolate values

    Args:
        path (str): The path to the CSV file
        precision (int, optional): The number of decimal places to which the probabilities are rounded

    Returns:
        segments (list of float): The segments from the error distribution
        probabilities (list of float): The corresponding probability of each segment
    """
    _, originalProbs = loadErrorValues(path, bins=bins, precision=precision)
    symmProbs = []
    # Get probabilities of segments
    # symmProbs = [
    #     originalProbs[4] / 2 + originalProbs[3] / 2,
    #     originalProbs[1] / 2 + originalProbs[2] / 2,
    #     originalProbs[0],
    #     originalProbs[1] / 2 + originalProbs[2] / 2,
    #     originalProbs[4] / 2 + originalProbs[3] / 2,
    # ]
    for i in range(len(originalProbs) - 1, 0, -2):
        symmProbs.append(originalProbs[i] / 2 + originalProbs[i - 1] / 2)

    reversedSymmProbs = symmProbs[::-1]
    symmProbs.append(originalProbs[0])
    symmProbs.extend(reversedSymmProbs)

    # get  segments
    segments, _ = loadErrorValues(
        path, bins=int(np.ceil(bins / 2)), precision=precision
    )
    newSegments = segments[::-1] * (-1)
    newSegments = np.concatenate((newSegments, segments[1:]), axis=0)

    try:
        rostopic.get_topic_class("/rosout")
        rospy.logdebug(
            f"[Monitored Space Observer] Loaded error distribution from {path}.\nSegments: {segments}\nProbabilities of segments: {symmProbs}"
        )
    except:
        print(
            f"[Monitored Space Observer] Loaded error distribution from {path}.\nSegments: {segments}\nProbabilities of segments: {symmProbs}"
        )

    return newSegments, symmProbs


def loadErrorDistributionLIDAR():
    segmentsAngles = [-0.17, -0.08, 0, 0.08, 0.17]
    probabilitiesAngles = [0.0153, 0.175, 0.7, 0.10, 0.008]
    segmentsDist = [-0.063, -0.037, -0.012, 0.012, 0.03]
    probabilitiesDist = [0.095, 0.2575, 0.29, 0.22, 0.12]

    try:
        rostopic.get_topic_class("/rosout")
        rospy.logdebug(
            f"[Crash and Remove] Loaded error distribution with LIDAR values."
        )
    except:
        print(f"[Crash and Remove] Loaded error distribution with LIDAR values.")
    return segmentsAngles, probabilitiesAngles, segmentsDist, probabilitiesDist


if __name__ == "__main__":
    # Just for testing or manually generating plots and error distributions
    segments, probs = generateErrorDistributionDistance(
        "/home/ros/catkin_ws/src/robotino/logs/error_dist_csvs/dumps/localization_error_dist_abend_guteslicht.csv",
        bins=5,
    )
    print(f"Segments: {segments}")
    print(f"Probs:: {probs}")
    plotErrorDist(
        "Fehlerverteilung Distanz",
        "Abweichung Distanz [m]",
        "/home/ros/catkin_ws/src/robotino/logs/error_dist_csvs/hist/test_5bins.png",
        segments,
        probs,
    )
