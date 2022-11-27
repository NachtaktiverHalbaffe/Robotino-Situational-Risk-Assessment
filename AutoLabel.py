import sklearn
from numpy import where, unique
# synthetic classification dataset
from sklearn.datasets import make_classification
from matplotlib import pyplot
import numpy as np
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
def plot_data(X,y,n_features):
    # create scatter plot for samples from each class
    for class_value in range(n_features):
        # get row indexes for samples with this class
        row_ix = where(y == class_value)
        # create scatter of these samples
        pyplot.scatter(X[row_ix, 0], X[row_ix, 1])
    # show the plot
    pyplot.show()

def create_model():
    model = []
    #model.append(AffinityPropagation)
    return model[0]

def plot_clusters(X, clusters, yhat):
    # create scatter plot for samples from each cluster
    for cluster in clusters:
        # get row indexes for samples with this cluster
        row_ix = where(yhat == cluster)
        # create scatter of these samples
        pyplot.scatter(X[row_ix, 0], X[row_ix, 1])
    # show the plot
    pyplot.show()

def affinity(X,y, plot=True):
    # https://www.science.org/doi/10.1126/science.1136800
    from sklearn.cluster import AffinityPropagation
    """ Affinity Propagation model"""
    model = AffinityPropagation(damping=0.6)
    model.fit(X)
    # assign a cluster to each example
    yhat = model.predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X, clusters, yhat)
    
def agglomerative(X,y, plot=True):
    #https://en.wikipedia.org/wiki/Hierarchical_clustering
    from sklearn.cluster import AgglomerativeClustering
    # define the model
    model = AgglomerativeClustering(n_clusters=3)
    # fit model and predict clusters
    yhat = model.fit_predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)

def birch(X, plot=True):
    """ Balanced Iterative Reducing and Clustering using Hierarchies """
    from sklearn.cluster import Birch
    #define the model
    model = Birch(threshold=0.01, n_clusters=3)
    # fit model and predict clusters
    yhat = model.fit_predict(X, y=X[:,2])
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)


def DBSCAN(X,y, plot=True):
    """ Density-Based Spatial Clustering of Applications with Noise """
    from sklearn.cluster import DBSCAN
    # define the model
    model = DBSCAN(eps=0.45, min_samples=3)
    # fit model and predict clusters
    yhat = model.fit_predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)


def K_Means(X,y, plot=True):
    # define the model
    from sklearn.cluster import KMeans
    model = KMeans(n_clusters=5, n_init=50)
    # fit model and predict clusters
    yhat = model.fit_predict(X, sample_weight=X[:,2])
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)

def MiniBatch_KMeans(X,y, plot=True):
    from sklearn.cluster import MiniBatchKMeans

    # define the model
    model = MiniBatchKMeans(n_clusters=3)
    # fit model and predict clusters
    yhat = model.fit_predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)

def MeanShift(X,y, plot=True):
    """ Very Expensive"""
    from sklearn.cluster import MeanShift
    # define the model
    model = MeanShift()
    # fit model and predict clusters
    yhat = model.fit_predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)


def OPTICS(X,y, plot=True):
    """ Ordering Points To Identify the Clustering Structure"""
    from sklearn.cluster import OPTICS
    # define the model
    model = OPTICS(eps=0.8, min_samples=10)
    # fit model and predict clusters
    yhat = model.fit_predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)

def SpectralClustering(X,y, plot=True):
    from sklearn.cluster import SpectralClustering
    # define the model
    model = SpectralClustering(n_clusters=5)
    # fit model and predict clusters
    yhat = model.fit_predict(X)
    # retrieve unique clusters
    clusters = unique(yhat)
    print(yhat)
    plot_clusters(X,clusters,yhat)

def MixtureofGaussians(X,y, plot=True):
    from sklearn.mixture import GaussianMixture
    # define the model
    model = GaussianMixture(n_components=3)
    # fit model and predict clusters
    yhat = model.fit_predict(X, y=X[:,2])
    print(yhat)
    # retrieve unique clusters
    clusters = unique(yhat)

    plot_clusters(X,clusters,yhat)

def main(data=None):
    # define dataset
    X, y = make_classification(n_samples=1000, n_features=2, n_informative=2, n_redundant=0, n_clusters_per_class=1, random_state=4)
    #print(X,y)
    #plot_data(X,y,2)
    #affinity(data,y=None)
    #agglomerative(data,y=None)
    #birch(data)
    #DBSCAN(data,y=None)
    K_Means(data,y=None)
    # MiniBatch_KMeans(data,y=None)
    #MeanShift(data,y=None)
    # OPTICS(data,y=None)
    # SpectralClustering(data,y=None)
    #MixtureofGaussians(data,y=None)

def read_data(filename):
    import pandas as pd
    df = pd.read_csv(filename, sep=',', header=None)
    #print(df.values)
    return df.values


if __name__ == '__main__':
    data = read_data("collision_data.csv")
    main(data)
    #read_data("collision_data.csv")
