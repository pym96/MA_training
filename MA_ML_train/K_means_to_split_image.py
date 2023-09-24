'''
 Author: Pan 
 Remember, using k-means in this way segments the image based on colors,
 not necessarily spatial structures. So regions with similar colors might be grouped together even if they're separated spatially.
 If you want spatially-aware segmentation, algorithms like SLIC (Simple Linear Iterative Clustering) or watershed might be more appropriate.
'''

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from skimage import io

# 1. Read the image
image = io.imread('/home/dan/learn/MA_training/MA_ML_train/demo.jpg')
io.imshow(image)
plt.show()

# 2. Reshape the image to be a list of RGB values
pixels = image.reshape(-1, 3)

# 3. Apply k-means to segment the RGB values
n_clusters = 25 # for example
kmeans = KMeans(n_clusters=n_clusters).fit(pixels)
segmented_img = kmeans.cluster_centers_[kmeans.labels_]

# 4. Reshape the clustered labels back into the image shape
segmented_img = segmented_img.reshape(image.shape).astype(np.uint8)

# 5. Display the segmented image
io.imshow(segmented_img)
plt.show()
