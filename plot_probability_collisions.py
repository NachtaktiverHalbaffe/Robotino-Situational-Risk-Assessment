import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from AutoLabel import evaluate_virtual_vs_ida

risk_brute_pos = [31.67, 42.3]
risk_brute_angle = [34.5, 44.1]
risk_ida_pos = [36.2, 42.5]
risk_ida_angle = [30.2, 44.5]


def read_data(filename):
    df = pd.read_csv(filename, sep=",")
    # print(df.values)
    # print(df)
    return df.values, df


def generate_synthetic_data(data):
    from sklearn.isotonic import IsotonicRegression

    total_entries = 100
    X_nD = data_orig[["N_nodes", "length"]].values[0:total_entries]
    X_1D = data_orig["length"].values[0:total_entries]
    X_1D_n = data_orig["length"].values
    Y_1D = data_orig["Prob_collision_real_world"].values[0:total_entries]

    """ Isotonic Regression"""
    ir = IsotonicRegression(out_of_bounds="clip")
    y_ = ir.fit_transform(X_1D, Y_1D)

    y__ = ir.predict(X_1D_n)
    # print(y__)

    """ SVR"""
    from sklearn.svm import SVR

    regr = SVR()
    regr.fit(X_nD, Y_1D)
    y_pred = regr.predict(X_nD)

    evaluate_virtual_vs_ida(Y_1D, y_)
    evaluate_virtual_vs_ida(Y_1D, y_pred)

    return y__


def plot_collsions(data_orig):

    path = "logs/evaluation/figures/"

    total_entries = 100
    print("Plotting")
    length_raw = data_orig["length"].values[0:total_entries]
    y_brute = data_orig["Prob_collision_Brute_force"].values[0:total_entries]
    y_ida = data_orig["Prob_collision_IDA"].values[0:total_entries]
    y_exp = data_orig["Expected Probability Collision"].values[0:total_entries]
    y_real_world = data_orig["Prob_collision_real_world"].values[0:total_entries]
    y_cal_error = data_orig["Prob_collision_robotino_calibration_error"].values[
        0:total_entries
    ]
    y_lights_error = data_orig["Prob_collision_robotino_lights_error"].values[
        0:total_entries
    ]
    start_x = data_orig["StartX"].values[0:total_entries]
    start_y = data_orig["StartY"].values[0:total_entries]
    end_x = data_orig["EndX"].values[0:total_entries]
    end_y = data_orig["EndY"].values[0:total_entries]
    N_nodes_raw = data_orig["N_nodes"].values[0:total_entries]
    risk = data_orig["Risk"].values[0:total_entries]
    y_pred_real_world_raw = generate_synthetic_data(data_orig)[0:total_entries]
    # y_pred_cal_error_raw = generate_synthetic_data(data_orig)[0:total_entries]
    # y_pred_lights_error_raw = generate_synthetic_data(data_orig)[0:total_entries]
    y_raw_brute = y_brute
    y_brute = y_brute + length_raw * 0.0013
    y_ida = y_ida + length_raw * 0.0013
    y_exp = y_exp + length_raw * 0.0013

    # print(y_real_world)
    N_nodes_1 = np.array(N_nodes_raw)
    length_1 = np.array(length_raw)
    prob_collision_brute_force_1 = np.array(y_brute)
    prob_collision_real_1 = np.array(y_real_world)
    prob_collision_ida_1 = np.array(y_ida)
    prob_collision_expected_1 = np.array(y_exp)
    y_pred_real_world_1 = np.array(y_pred_real_world_raw)
    prob_collision_cal_error_1 = np.array(y_cal_error)
    prob_collision_lights_error_1 = np.array(y_lights_error)
    prob_collision_brute_force_old = np.array(y_raw_brute)
    risk_sorted = np.array(risk)

    indices = np.argsort(length_1)

    # N_nodes = np.argsort(N_nodes_1)
    length = length_1[indices]
    prob_collision_brute_force = prob_collision_brute_force_1[indices]
    prob_collision_real = prob_collision_real_1[indices]
    prob_collision_ida = prob_collision_ida_1[indices]
    prob_collision_expected = prob_collision_expected_1[indices]
    N_nodes = N_nodes_1[indices]
    y_pred_real_world = y_pred_real_world_1[indices]
    prob_collision_cal_error = prob_collision_cal_error_1[indices]
    prob_collision_lights_error = prob_collision_lights_error_1[indices]
    prob_collision_brute_old = prob_collision_brute_force_old[indices]
    risk_sorted = risk_sorted[indices]

    avg_window = 25
    n_windows = int(len(length) / avg_window)
    length_avg = []
    prob_collision_brute_force_avg = []
    prob_collision_real_avg = []
    prob_collision_ida_avg = []
    prob_collision_expected_avg = []
    N_nodes_avg = []
    y_pred_real_world_avg = []
    prob_collision_cal_error_avg = []
    prob_collision_lights_error_avg = []
    prob_collision_brute_old_avg = []
    risk_avg = []

    # calculate average reward
    for i in range(0, n_windows):
        length_window = length[(i * avg_window) : (i * avg_window + avg_window)]
        brute_force_window = prob_collision_brute_force[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        real_window = prob_collision_real[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        ida_window = prob_collision_ida[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        expected_window = prob_collision_expected[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        N_nodes_window = N_nodes[(i * avg_window) : (i * avg_window + avg_window)]
        y_pred_real_world_window = y_pred_real_world[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        prob_collision_cal_error_window = prob_collision_cal_error[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        prob_collision_lights_error_window = prob_collision_lights_error[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        prob_collision_brute_old_window = prob_collision_brute_old[
            (i * avg_window) : (i * avg_window + avg_window)
        ]
        risk_window = risk_sorted[(i * avg_window) : (i * avg_window + avg_window)]

        length_avg.append(np.mean(length_window))
        prob_collision_brute_force_avg.append(np.mean(brute_force_window))
        prob_collision_real_avg.append(np.mean(real_window))
        prob_collision_ida_avg.append(np.mean(ida_window))
        prob_collision_expected_avg.append(np.mean(expected_window))
        N_nodes_avg.append(np.mean(N_nodes_window))
        y_pred_real_world_avg.append(np.mean(y_pred_real_world_window))
        prob_collision_cal_error_avg.append(np.mean(prob_collision_cal_error_window))
        prob_collision_lights_error_avg.append(
            np.mean(prob_collision_lights_error_window)
        )
        prob_collision_brute_old_avg.append(np.mean(prob_collision_brute_old_window))
        risk_avg.append(np.mean(risk_window))

    # x = list(range(0, n_windows))
    # x = np.array(x)*avg_window

    # data, data_orig = read_data("collision_data.csv")
    # prob_brute_robot_big = plot_collision_robot_big(data_orig)

    """ Plots for Probability of collision for different scenarios"""
    fig, ax = plt.subplots()

    plt.figure(1)

    ax.plot(length_avg, prob_collision_brute_old_avg, label="Brute force 48 cm")
    ax.plot(
        length_avg, prob_collision_brute_force_avg, "r--", label="Brute force 64 cm"
    )
    ax.plot(length_avg, prob_collision_ida_avg, "g", label="IDA 64cm")
    ax.plot(length_avg, prob_collision_expected_avg, "b", label="Expected 64cm")
    ax.plot(length_avg, prob_collision_real_avg, "c", label="Real world")
    # ax.plot(length_avg, y_pred_real_world_avg, 'm', label='real world pred')
    ax.plot(length_avg, prob_collision_cal_error_avg, "y", label="Calibration error")
    ax.plot(length_avg, prob_collision_lights_error_avg, "k", label="Lights error")

    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_title("Probability of collisions")
    ax.set_ylabel("Prob. Collision")
    ax.set_xlabel("length of trajectory")

    plt.savefig(path + "Probability of collisions for different scenarios.png")

    """ Bar chart for probability of collsions"""
    fig, ax = plt.subplots()
    plt.figure(2)

    ax.bar(
        list(np.asarray(length_avg) + 0),
        prob_collision_brute_old_avg,
        width=4,
        label="Brute force 48 cm",
    )
    ax.bar(
        list(np.asarray(length_avg) + 4),
        prob_collision_brute_force_avg,
        width=4,
        label="Brute force 64 cm",
    )
    ax.bar(
        list(np.asarray(length_avg) + 8),
        prob_collision_ida_avg,
        width=4,
        label="IDA 64 cm",
    )
    ax.bar(
        list(np.asarray(length_avg) + 12),
        prob_collision_expected_avg,
        width=4,
        label="Exp. 64 cm",
    )
    ax.bar(
        list(np.asarray(length_avg) + 16),
        prob_collision_real_avg,
        width=4,
        label="Real world",
    )
    ax.bar(
        list(np.asarray(length_avg) + 20),
        prob_collision_cal_error_avg,
        width=4,
        label="Calibration error",
    )
    ax.bar(
        list(np.asarray(length_avg) + 24),
        prob_collision_lights_error_avg,
        width=4,
        label="Lights error",
    )

    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_title("Bar chart for probability of collsions")
    ax.set_ylabel("Prob. Collision")
    ax.set_xlabel("length of trajectory")

    plt.savefig(
        path + "Bar chart for probability of collsions for different scenarios.png"
    )

    """ Box plot for risk (Upper and lower bound) and for each map, methods and Position and Angle"""
    fig, ax = plt.subplots()
    plt.figure(3)
    y_label = ["0", "Brute Angle", "Brute Pos", "IDA Angle", "IDA Pos"]
    data = [
        np.array(risk_brute_angle),
        np.array(risk_brute_pos),
        np.array(risk_ida_angle),
        np.array(risk_ida_pos),
    ]
    box = plt.boxplot(
        data, showmeans=True, meanline=True, vert=False, patch_artist=True
    )
    colors = ["royalblue", "lightblue", "lightgreen", "pink"]
    for patch, color in zip(box["boxes"], colors):
        patch.set_facecolor(color)
    ax.legend(
        [box["boxes"][0], box["boxes"][1], box["boxes"][2], box["boxes"][3]],
        ["Brute Angle", "Brute Pos", "IDA Angle", "IDA Pos"],
        loc="upper left",
    )
    ax.set_title("Risk (Upper and lower bound)")
    ax.set_ylabel("Methods")
    ax.set_xlabel("Risk")
    y_pos = np.arange(5)
    plt.yticks(y_pos, y_label, rotation=45, horizontalalignment="right")
    ax.set_title("Box plot for risk")
    plt.savefig(path + "Box plot for risk.png")

    """ Plot trajectory on a map for 'Original', 'Robot', 'Calibration Error', 'Lights error' """
    # Draw_trajectory.py

    """ 3D plot for prob_collision, length, nodes"""
    fig = plt.figure(4)
    ax = fig.add_subplot(projection="3d")
    ax.scatter(length_raw, N_nodes_1, y_brute, marker="o", label="Brute Force")
    ax.scatter(length_raw, N_nodes_1, y_real_world, marker="^", label="Real World")
    ax.scatter(length_raw, N_nodes_1, y_lights_error, marker="v", label="Lights Error")
    ax.scatter(
        length_raw, N_nodes_1, y_cal_error, marker=",", label="Calibration Error"
    )
    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_xlabel("Length")
    ax.set_ylabel("Nodes")
    ax.set_zlabel("Prob. Collision")

    ax.set_title("3D plot for prob_collision, length, nodes")
    plt.savefig(path + "3D plot for prob_collision length nodes.png")

    """ 3D plot for startx, endx, prob_collsion"""
    fig = plt.figure(5)
    ax = fig.add_subplot(projection="3d")
    ax.scatter(start_x, start_y, y_brute, marker="o", label="Brute Force")
    ax.scatter(start_x, start_y, y_real_world, marker="^", label="Real World")
    ax.scatter(start_x, start_y, y_lights_error, marker="v", label="Lights Error")
    ax.scatter(start_x, start_y, y_cal_error, marker=",", label="Calibration Error")
    # Todo: for other prob collisions

    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_xlabel("StartX")
    ax.set_ylabel("StartY")
    ax.set_zlabel("Prob. Collision")
    ax.set_title("3D plot for startx, endx, prob_collsion")
    plt.savefig(path + "3D plot for startx endx prob_collsion.png")

    """ 3D plot for endx, endx, prob_collsion"""
    fig = plt.figure(6)
    ax = fig.add_subplot(projection="3d")
    ax.scatter(end_x, end_y, y_brute, marker="o", label="Brute Force")
    ax.scatter(end_x, end_y, y_real_world, marker="^", label="Real World")
    ax.scatter(end_x, end_y, y_lights_error, marker="v", label="Lights Error")
    ax.scatter(end_x, end_y, y_cal_error, marker=",", label="Calibration Error")
    # Todo: for other prob collisions
    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_xlabel("EndX")
    ax.set_ylabel("EndY")
    ax.set_zlabel("Prob. Collision")
    ax.set_title("3D plot for endx, endx, prob_collsion")
    plt.savefig(path + "3D plot for endx endx prob_collsion.png")

    """ Search space Start Positions"""
    fig = plt.figure(7)
    ax = fig.add_subplot()
    ax.scatter(
        data_orig["StartX"].values,
        data_orig["StartY"],
        marker="o",
        label="Start Positions",
    )
    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_xlabel("StartX")
    ax.set_ylabel("StartY")
    ax.set_title("Search space Start Positions")
    plt.savefig(path + "Search space Start Positions.png")

    """ Search space End Positions"""
    fig = plt.figure(8)
    ax = fig.add_subplot()
    ax.scatter(
        data_orig["EndX"].values, data_orig["EndY"], marker="^", label="End Position"
    )
    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_xlabel("EndX")
    ax.set_ylabel("EndY")

    ax.set_title("Search space End Positions")
    plt.savefig(path + "Search space End Positions.png")

    """ 3D plot for Risk, length, nodes"""
    fig = plt.figure(9)
    ax = fig.add_subplot(projection="3d")
    ax.scatter(length_raw, N_nodes_1, risk, marker="o", label="Risk")
    ax.legend(loc="upper left", shadow=False, fontsize="small")
    ax.set_xlabel("Length")
    ax.set_ylabel("Nodes")
    ax.set_zlabel("Risk")
    ax.set_title("3D plot for length, nodes and risk")
    plt.savefig(path + "3D plot for length nodes and risk.png")

    """ Distribution plots"""
    fig, ax = plt.subplots()
    plt.figure(10)
    sns.kdeplot(
        prob_collision_brute_force, fill=True, color="g", label="Brute Force", alpha=0.7
    )
    sns.kdeplot(
        prob_collision_ida + 2, fill=True, color="deeppink", label="IDA", alpha=0.7
    )
    # sns.kdeplot(prob_collision_expected, fill=True, color="dodgerblue", label="Expected", alpha=.7)
    sns.kdeplot(
        y_real_world + 4, fill=True, color="dodgerblue", label="Real World", alpha=0.7
    )
    # sns.kdeplot(y_cal_error, fill=True, color="lime", label="Calibration Disturbances", alpha=.7)
    # sns.kdeplot(y_lights_error, fill=True, color="cyan", label="Lights Disturbances", alpha=.7)
    # sns.displot([prob_collision_expected, prob_collision_ida], kind="kde", fill=True)
    plt.legend(loc="upper left")
    ax.set_xlabel("Probabilty of collision")
    ax.set_ylabel("Density")
    ax.set_title("Distribution vs Density plots")
    plt.savefig(path + "Distribution vs Density plots.png")
    plt.show()


def plot_collision_robot_big(data_orig):
    total_entries = 100
    print("Plotting")
    length_raw = data_orig["length"].values[0:total_entries]
    y_brute = data_orig["Prob_collision_Brute_force"].values[0:total_entries]

    length_1 = np.array(length_raw)
    prob_collision_brute_force_1 = np.array(y_brute)

    indices = np.argsort(length_1)

    # N_nodes = np.argsort(N_nodes_1)
    length = length_1[indices]
    prob_collision_brute_force = prob_collision_brute_force_1[indices]

    avg_window = 20
    n_windows = int(len(length) / avg_window)
    length_avg = []
    prob_collision_brute_force_avg = []

    # calculate average reward
    for i in range(0, n_windows):
        length_window = length[(i * avg_window) : (i * avg_window + avg_window)]
        brute_force_window = prob_collision_brute_force[
            (i * avg_window) : (i * avg_window + avg_window)
        ]

        length_avg.append(np.mean(length_window))
        prob_collision_brute_force_avg.append(np.mean(brute_force_window))

    fig, ax = plt.subplots()

    plt.figure(1)

    ax.plot(length_avg, prob_collision_brute_force_avg, "r--", label="brute force")
    plt.ylim(0, 1.0)

    plt.show()
    return prob_collision_brute_force_avg


if __name__ == "__main__":
    data, data_orig = read_data(
        "logs/evaluation/data/collision_data_exp_ida_brute_force_angle.csv"
    )
    # print(data[1:-1,5:8])
    length = data_orig["length"].values
    y_brute = data_orig["Prob_collision_Brute_force"].values
    y_ida = data_orig["Prob_collision_IDA"].values
    y_exp = data_orig["Expected Probability Collision"].values
    y_real_world = data_orig["Prob_collision_real_world"].values
    start_x = data_orig["StartX"].values
    start_y = data_orig["StartY"].values
    end_x = data_orig["EndX"].values
    end_y = data_orig["EndY"].values

    plot_collsions(data_orig)

    data, data_orig = read_data("logs/collision_data_00.csv")
    # plot_collision_robot_big(data_orig)

    # generate_synthetic_data(data_orig)
    # print(data_orig)
    # main()
