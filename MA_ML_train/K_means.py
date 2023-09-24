'''
Author: Pan && Gpt-4 AI father
The k-means algorithm is a popular clustering algorithm that aims to partition a set of points into k clusters, where each point belongs to the cluster with the nearest mean (or centroid).

Here's a step-by-step breakdown of the k-means algorithm:

    Initialization: Randomly select k data points as initial centroids.
    Assignment step: Assign each data point to the closest centroid.
    Update step: Calculate the new centroids as the mean of the data points in each cluster.
    Convergence: Repeat the assignment and update steps until the centroids no longer change significantly.

'''

import numpy as np

def kmeans(X, k, max_iters=100, tolerance=1e-4):
    # 1. Initialization: Randomly choose k data points as initial centroids
    centroids = X[np.random.choice(X.shape[0], k, replace=False)]
    
    prev_centroids = np.zeros(centroids.shape)
    clusters = np.zeros(X.shape[0])
    
    # Main loop
    for i in range(max_iters):
        # 2. Assignment step: Assign each data point to the closest centroid
        for idx, data in enumerate(X):
            distances = np.linalg.norm(data - centroids, axis=1)
            cluster = np.argmin(distances)
            clusters[idx] = cluster
        
        prev_centroids = centroids.copy()
        
        # 3. Update step: Calculate the new centroids
        for cluster_num in range(k):
            points_in_cluster = [X[j] for j in range(X.shape[0]) if clusters[j] == cluster_num]
            centroids[cluster_num] = np.mean(points_in_cluster, axis=0)
        
        # 4. Convergence: Check if centroids have changed significantly
        if np.linalg.norm(centroids - prev_centroids) < tolerance:
            break
    
    return clusters, centroids

# Sample usage
X = np.array([[1, 2],
              [5, 6],
              [1, 1],
              [5, 5],
              [2, 3],
              [6, 6]])

clusters, centroids = kmeans(X, 2)
print("Clusters:", clusters)
print("Centroids:", centroids)
