import numpy as np
import matplotlib.pyplot as plt
from skimage import io
from skimage.segmentation import slic
from skimage.color import label2rgb
from skimage.segmentation import mark_boundaries

# 1. Load the image
image = io.imread('/home/dan/learn/MA_training/MA_ML_train/demo.jpg')

# 2. Apply the SLIC algorithm to generate superpixels
segments = slic(image, n_segments=5, compactness=10, sigma=1)

# Visualizing the SLIC segmented image with boundaries
fig, axarr = plt.subplots(1, 2, figsize=(15, 10))

# Original image
axarr[0].imshow(image)
axarr[0].set_title("Original Image")
axarr[0].axis('off')

# Superpixel segmentation with boundaries
axarr[1].imshow(mark_boundaries(image, segments))
axarr[1].set_title("SLIC Superpixel Segmentation")
axarr[1].axis('off')

plt.tight_layout()
plt.show()
