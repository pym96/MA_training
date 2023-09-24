import numpy as np
import matplotlib.pyplot as plt

# Step 1: Generate synthetic data
np.random.seed(42)  # for reproducibility

cluster_1 = np.random.normal(loc=0, scale=1, size=(100, 2))
cluster_2 = np.random.normal(loc=5, scale=1, size=(100, 2))
cluster_3 = np.random.normal(loc=10, scale=1.5, size=(100, 2))
cluster_4 = np.random.normal(loc=-5, scale=0.5, size=(100, 2))

data = np.vstack([cluster_1, cluster_2, cluster_3, cluster_4])

# Step 2: Implement k-means
def kmeans(X, k, max_iters=10):
    centroids = X[np.random.choice(X.shape[0], k, replace=False)]
    for i in range(max_iters):
        distances = np.linalg.norm(X[:, np.newaxis] - centroids, axis=2)
        labels = np.argmin(distances, axis=1)
        new_centroids = np.array([X[labels == j].mean(axis=0) for j in range(k)])
        if np.all(centroids == new_centroids):
            break
        centroids = new_centroids
    return labels, centroids

labels, centroids = kmeans(data, 4)

# Step 3: Visualization
plt.figure(figsize=(15, 5))

# Original data points and initial centroids
plt.subplot(1, 3, 1)
plt.scatter(data[:, 0], data[:, 1], alpha=0.5)
plt.scatter(centroids[:, 0], centroids[:, 1], c='red', marker='x')
plt.title("Original Data & Initial Centroids")

# Clustered data
plt.subplot(1, 3, 2)
plt.scatter(data[:, 0], data[:, 1], c=labels, alpha=0.5)
plt.scatter(centroids[:, 0], centroids[:, 1], c='red', marker='x')
plt.title("Clustered Data")

# Clustered data with centroids highlighted
plt.subplot(1, 3, 3)
for i, color in enumerate(['blue', 'green', 'yellow', 'cyan']):
    plt.scatter(data[labels == i][:, 0], data[labels == i][:, 1], c=color, alpha=0.5)
    plt.scatter(centroids[i, 0], centroids[i, 1], c='red', s=100, marker='x')
plt.title("Clustered Data & Centroids")

plt.tight_layout()
plt.show()
