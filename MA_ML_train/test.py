
import numpy as np

X = np.array([[1, 2],
              [5, 6],
              [1, 1],
              [5, 5],
              [2, 3],
              [6, 6],
              [1, 4]])

def kmeans(X, k, max_iters=100, tolerance=1e-4):
    # 1. Initialization: Randomly choose k data points as initial centroids
    centroids = X[np.random.choice(X.shape[0], k, replace=False)]
  


centroids = X[np.random.choice(X.shape[0], 2, replace=False)]
cluster_1 = np.random.normal(loc=0, scale=1, size=(100, 2))

print(np.array([1,2,3]).cumsum().tolist())
