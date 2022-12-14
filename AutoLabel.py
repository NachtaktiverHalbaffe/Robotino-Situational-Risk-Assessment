import sklearn
from numpy import where, unique

# synthetic classification dataset
from sklearn.datasets import make_classification
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.metrics import silhouette_score

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
def plot_data(X, y, n_features):
    # create scatter plot for samples from each class
    for class_value in [0, 0.5]:
        # get row indexes for samples with this class
        row_ix = where(y >= class_value)
        # create scatter of these samples
        plt.scatter(X[row_ix, 0], X[row_ix, 2])
    # show the plot
    plt.show()


def main(data=None):
    # define dataset
    X, y = make_classification(
        n_samples=1000,
        n_features=3,
        n_informative=2,
        n_redundant=0,
        n_clusters_per_class=1,
        random_state=4,
    )
    # print(X,y)
    # plot_3d(data)

    # plot_data(data,data[:,2],2)
    # affinity(data,y=None)
    # agglomerative(data,y=None)
    # birch(data)
    # DBSCAN(data,y=None)
    # K_Means(data,y=None)
    # MiniBatch_KMeans(data,y=None)
    # MeanShift(data,y=None)
    # OPTICS(data,y=None)
    # SpectralClustering(data,y=None)
    # MixtureofGaussians(data,y=None)


def predict_collision(data, y=None):
    from sklearn.linear_model import LinearRegression

    pass


class AutoLabel:
    def __init__(self, X, y=None, n_clusters=3, method=0, plot=True):
        self.clusters = n_clusters
        self.method = method
        self.X = X
        self.plot = plot

    #     self.algorithms = [K_Means(), MiniBatch_KMeans(), MixtureofGaussians(), agglomerative()]

    # def create_clusters(self):
    #     detection = self.algorithms[self.method]

    def plot_clusters(self, X, clusters, yhat):
        # create scatter plot for samples from each cluster
        for cluster in clusters:
            # get row indexes for samples with this cluster
            row_ix = where(yhat == cluster)
            # create scatter of these samples
            plt.scatter(X[row_ix, 1], X[row_ix, 2])
        # show the plot
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


def read_data(filename):
    df = pd.read_csv(filename, sep=",", header=None)
    # print(df.values)
    return df.values, df


if __name__ == "__main__":
    data, data_orig = read_data("collision_data.csv")
    # data = data_orig.values
    linear_regression(data)
    # print (data)
    # print(data[:,0:2])
    # print(data[:,2])
    # print(where(data[:,2] >= 0.5))
    # main(data)
    al = AutoLabel(data, n_clusters=3, method=0)
    yhat = al.MixtureofGaussians()
    al.K_Means()
    # data_orig.insert(3, "label", yhat, True)
    # print(data_orig)
    # data_orig.to_csv('labelled_data.csv', header=False, index=False)

    # print(data[:,0:2])
    # read_data("collision_data.csv")
