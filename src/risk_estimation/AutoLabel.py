import sklearn
from numpy import where, unique

# synthetic classification dataset
from sklearn.datasets import make_classification
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.metrics import silhouette_score
import seaborn as sns

"""
    A list of 10 of the more popular algorithms is as follows:

    Affinity Propagation
    Agglomerative Clustering
    BIRCH
    DBSCAN
    K-Means
    Mini-Batch K-Means
    Mean Shift
    OPTICS
    Spectral Clustering
    Mixture of Gaussians
"""
# figure, axis = pyplot.subplots(2, 2)
def plot_data(X, y):
    """
    Create scatter plot for samples from each class and shows it

    Args:
        x: Data to plot
        y:
    """
    for class_value in [0, 0.5]:
        # get row indexes for samples with this class
        row_ix = where(y >= class_value)
        # create scatter of these samples
        plt.scatter(X[row_ix, 0], X[row_ix, 2])
    # show the plot
    plt.show()


def predict_collision(data, y=None):
    from sklearn.linear_model import LinearRegression

    pass


class AutoLabel:
    """
    Class for automatic labeling of data. Implemented methods are:
        - Agglomerative
        - K-Means
        - Mini-Batch K-Means
        - Mixture of Gaussian
    """

    def __init__(self, X, y=None, n_clusters=3, method=0, plot=True):
        self.clusters = n_clusters
        """ int: Number of clusters into which the data should be labeled """
        self.method = method
        """ int: Method which should be chosen. Is specified with the index of the according list\
             [K_Means(), MiniBatch_KMeans(), MixtureofGaussians(), agglomerative()]. Defaults to 0 """
        self.X = X
        """Data which should be clustered"""
        self.plot = plot
        self.path = "evaluation/figures/"
        self.y_real = prob_real_world

    def plot_clusters(self, X, clusters, yhat):
        """
        Create scatter plot for samples from each cluster

        Args:
            X: Data to plot
            clusters:
            yhat:
        """
        self.path = "logs/evaluation/figures/"
        prob_collision = []
        mean_ar = []
        for cluster in clusters:
            # get row indexes for samples with this cluster
            row_ix = where(yhat == cluster)
            plt.scatter(X[row_ix, 2], X[row_ix, 0])
            prob_collision.append(X[row_ix, 2])
            # mean_ar.append(np.mean(X[row_ix,2]))
            print(self.y_real[row_ix])
            mean_ar.append(np.mean(self.y_real[row_ix]))
            print(np.mean(X[row_ix, 2]))
        plt.xlabel("Probability of collisions")
        plt.ylabel("Length of Trajectory")
        plt.title("Auto Labeling")
        # show the plot
        fig, ax = plt.subplots()
        plt.figure(2)
        mean_ind = np.argsort(mean_ar)
        color = ["g", "c", "r", "m", "y"]
        Risk = ["Low", "Medium", "High"]
        for i in range(len(mean_ind)):
            n, bins, patches = ax.hist(
                prob_collision[mean_ind[i]][0] + i * 1,
                density=True,
                facecolor=color[i],
                alpha=0.75,
                label=Risk[i],
            )
            ax.arrow(
                mean_ar[mean_ind[i]] + i * 1,
                2,
                0,
                -1.7,
                head_width=0.1,
                head_length=0.3,
                fc="k",
                ec="k",
            )
            plt.text(
                mean_ar[mean_ind[i]] + i * 1,
                2,
                r"$\mu=$" + str(np.round(mean_ar[mean_ind[i]], 2)),
            )
        ax.legend(loc="upper right", shadow=False, fontsize="small")
        ax.set_xlabel("Probability of collisions")
        ax.set_ylabel("Density")
        # y_label = ['0', '0.4', '0.4', '0.375', '0.5', '0.625', '0.75', '1.0']
        y_label = ["0", "1.0"]
        # y_label = np.arange(0,1.0,0.5)
        # y_label= ["%.3f" % x for x in y_label]
        # y_pos = [0,0.125,38,42,45,48,51,54]
        # y_pos = np.arange(0,3.5,1.75)
        y_pos = [0, 3]
        plt.xticks(y_pos, y_label, horizontalalignment="right")
        # plt.grid(True)
        plt.title("Histogram of Clusters with mean")
        plt.savefig(self.path + "Histogram of Clusters.png")

        fig, ax = plt.subplots()
        plt.figure(3)
        color = ["g", "deeppink", "r", "m", "y"]
        # #print(prob_collision[0])
        for i in range(len(mean_ind)):
            sns.kdeplot(
                prob_collision[mean_ind[i]][0] + i * 2,
                fill=True,
                color=color[i],
                label=Risk[i],
                alpha=0.7,
            )
            ax.arrow(
                mean_ar[mean_ind[i]] + i * 2,
                1.0,
                0,
                -0.9,
                head_width=0.2,
                head_length=0.1,
                fc="k",
                ec="k",
            )
            plt.text(
                mean_ar[mean_ind[i]] + i * 2,
                1.0,
                r"$\mu=$" + str(np.round(mean_ar[mean_ind[i]], 2)),
            )
        ax.legend(loc="upper right", shadow=False, fontsize="small")
        ax.set_xlabel("Probability of Collision")
        ax.set_ylabel("Density")
        y_label = ["0", "1.0"]
        # y_label = np.arange(0,1.0,0.5)
        # y_label= ["%.3f" % x for x in y_label]
        # y_pos = [0,0.125,38,42,45,48,51,54]
        # y_pos = np.arange(0,3.5,1.75)
        y_pos = [0, 5.25]
        plt.xticks(y_pos, y_label, rotation=45, horizontalalignment="right")
        plt.title("Density of Clusters with mean")
        plt.savefig(self.path + "Density of Clusters.png")
        plt.show()

    def agglomerative(self):
        # https://en.wikipedia.org/wiki/Hierarchical_clustering
        from sklearn.cluster import AgglomerativeClustering

        # define the model
        model = AgglomerativeClustering(n_clusters=self.clusters)
        # fit model and predict clusters
        yhat = model.fit_predict(self.X)
        # retrieve unique clusters
        clusters = unique(yhat)
        # print(yhat)
        if self.plot:
            self.plot_clusters(self.X, clusters, yhat)
        return yhat

    def K_Means(self):
        """
        Clusters the data with help of k-means
        """
        # define the model
        from sklearn.cluster import KMeans

        model = KMeans(n_clusters=self.clusters, n_init=100)
        # fit model and predict clusters
        yhat = model.fit_predict(self.X, sample_weight=self.X[:, -1])
        # retrieve unique clusters
        clusters = unique(yhat)
        # print(yhat)
        if self.plot:
            self.plot_clusters(self.X, clusters, yhat)
        score = silhouette_score(self.X, model.labels_)
        # print(model.labels_)
        # print(yhat)
        print(score)
        return yhat

    def MiniBatch_KMeans(self):
        from sklearn.cluster import MiniBatchKMeans

        # define the model
        model = MiniBatchKMeans(n_clusters=self.clusters)
        # fit model and predict clusters
        yhat = model.fit_predict(self.X)
        # retrieve unique clusters
        clusters = unique(yhat)
        # print(yhat)
        if self.plot:
            self.plot_clusters(self.X, clusters, yhat)
        return yhat

    def MixtureofGaussians(self):
        from sklearn.mixture import BayesianGaussianMixture

        # define the model
        model = BayesianGaussianMixture(
            n_components=self.clusters,
            covariance_type="tied",
            weight_concentration_prior_type="dirichlet_distribution",
            init_params="kmeans",
        )
        # fit model and predict clusters
        yhat = model.fit_predict(self.X)
        # print(yhat)
        # retrieve unique clusters
        clusters = unique(yhat)
        # score= silhouette_score(self.X, yhat)
        # print(score)
        if self.plot:
            self.plot_clusters(self.X, clusters, yhat)
        return yhat


def linear_regression(data):
    from sklearn.model_selection import train_test_split

    # Dropping any rows with Nan values
    X_train, X_test, y_train, y_test = train_test_split(
        data[:, 0:2], data[:, -1], test_size=0.2
    )

    print(len(X_train), len(X_test), len(y_test))
    from sklearn.linear_model import LinearRegression

    regr = LinearRegression()
    regr.fit(X_train, y_train)
    print(regr.score(X_test, y_test))

    from sklearn.metrics import mean_absolute_error, mean_squared_error

    y_pred = regr.predict(X_test)
    mae = mean_absolute_error(y_true=y_test, y_pred=y_pred)
    # squared True returns MSE value, False returns RMSE value.
    mse = mean_squared_error(y_true=y_test, y_pred=y_pred)  # default=True
    rmse = mean_squared_error(y_true=y_test, y_pred=y_pred, squared=False)

    print("MAE:", mae)
    print("MSE:", mse)
    print("RMSE:", rmse)


def svm_regression(data):
    from sklearn.model_selection import train_test_split

    # Dropping any rows with Nan values
    X_train, X_test, y_train, y_test = train_test_split(
        data[:, 0:6], data[:, -1], test_size=0.1
    )

    # from sklearn import svm
    from sklearn.svm import SVR

    # regr = svm.SVR()
    regr = SVR()

    regr.fit(X_train, y_train)
    print(regr.score(X_test, y_test))

    my_data = [[142, 78, 89, 57, 5.0, 57.185]]
    from sklearn.metrics import mean_absolute_error, mean_squared_error

    y_pred = regr.predict(X_test)
    print(regr.predict(my_data))

    mae = mean_absolute_error(y_true=y_test, y_pred=y_pred)
    # squared True returns MSE value, False returns RMSE value.
    mse = mean_squared_error(y_true=y_test, y_pred=y_pred)  # default=True
    rmse = mean_squared_error(y_true=y_test, y_pred=y_pred, squared=False)

    print("MAE:", mae)
    print("MSE:", mse)
    print("RMSE:", rmse)
    return regr.predict(data[:, 0:6]).round(4)


def ransac(data):
    from sklearn.model_selection import train_test_split

    # Dropping any rows with Nan values
    X_train, X_test, y_train, y_test = train_test_split(
        data[:, 0:6], data[:, -1], test_size=0.2
    )

    print(len(X_train), len(X_test), len(y_test))
    from sklearn.linear_model import ElasticNet

    regr = ElasticNet(random_state=0)
    regr.fit(X_train, y_train)
    print(regr.score(X_test, y_test))

    my_data = [[142, 78, 89, 57, 5.0, 57.185]]
    from sklearn.metrics import mean_absolute_error, mean_squared_error

    y_pred = regr.predict(X_test)
    print(regr.predict(my_data))

    mae = mean_absolute_error(y_true=y_test, y_pred=y_pred)
    # squared True returns MSE value, False returns RMSE value.
    mse = mean_squared_error(y_true=y_test, y_pred=y_pred)  # default=True
    rmse = mean_squared_error(y_true=y_test, y_pred=y_pred, squared=False)

    print("MAE:", mae)
    print("MSE:", mse)
    print("RMSE:", rmse)


def read_data(filename):
    df = pd.read_csv(filename, sep=",", header=None)
    # print(df.values)
    return df.values, df


def evaluate_virtual_vs_ida(y_true, y_pred):
    from sklearn.metrics import mean_absolute_error, mean_squared_error

    mae = mean_absolute_error(y_true=y_true, y_pred=y_pred)
    # squared True returns MSE value, False returns RMSE value.
    mse = mean_squared_error(y_true=y_true, y_pred=y_pred)  # default=True
    rmse = mean_squared_error(y_true=y_true, y_pred=y_pred, squared=False)

    print("MAE:", mae)
    print("MSE:", mse)
    print("RMSE:", rmse)
    return mae, mse, rmse


def calculate_risk(prob_collision, length, nodes):
    from sklearn.preprocessing import normalize

    length = normalize([length], norm="max")
    nodes = normalize([nodes], norm="max")
    risk_ida = [30.2, 44.5]
    r_max = risk_ida[1]
    r_min = risk_ida[0]

    weight = (prob_collision * 0.70) + (length * 0.15) + (nodes * 0.15)
    risk = r_min + (r_max - r_min) * weight

    return risk


if __name__ == "__main__":
    # data, data_orig = read_data("collision_data_000.csv")
    data, data_orig = read_data(
        "logs/evaluation/data/collision_data_exp_ida_brute_force_angle_cp.csv"
    )
    # print(data[1:-1,5:8])
    X_clustering = data_orig[
        ["length", "length", "Prob_collision_Brute_force_8_pix"]
    ].values[0:99]
    y_brute = data_orig["Prob_collision_Brute_force"].values
    y_ida = data_orig["Prob_collision_IDA"].values
    y_exp = data_orig["Expected Probability Collision"].values
    y_brute = data_orig["Prob_collision_Brute_force"].values
    length_raw = data_orig["length"].values
    nodes = data_orig["N_nodes"].values
    y_real = data_orig["Prob_collision_real_world"].values[0:99]

    al = AutoLabel(X_clustering, n_clusters=3, method=0, prob_real_world=y_real)
    yhat = al.K_Means()
    # al.K_Means()

    # data_orig.insert(7, "Expected Prob collision", yhat, True)
    # #print(data_orig)
    # data_orig.to_csv('labelled_data_pc.csv', header=False, index=False)

    # print(data[:,0:2])
    # read_data("collision_data.csv")
