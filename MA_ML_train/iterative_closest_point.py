import numpy as np

def closest_point(data, reference):
    """Find the closest point in 'reference' for each point in 'data'."""
    deltas = data[:, np.newaxis] - reference
    dist_2 = np.einsum('ijk,ijk->ij', deltas, deltas)
    return reference[np.argmin(dist_2, axis=1)]

def icp(data, reference, max_iterations=100, tolerance=1e-5):
    """Basic ICP algorithm."""
    prev_error = 0

    for i in range(max_iterations):
        # Find the closest points for each data point in the reference point cloud
        closest_points = closest_point(data, reference)

        # Compute the mean of the data and reference points
        mean_data = np.mean(data, axis=0)
        mean_reference = np.mean(closest_points, axis=0)

        # Subtract the means
        data_shifted = data - mean_data
        reference_shifted = closest_points - mean_reference

        # Calculate the rotation using Singular Value Decomposition
        H = np.dot(data_shifted.T, reference_shifted)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # Calculate the translation
        t = mean_reference.T - np.dot(R, mean_data.T)

        # Apply the transformation to the data point cloud
        data = np.dot(data, R) + t

        # Compute the mean squared error
        mean_error = np.mean(np.linalg.norm(data - closest_points, axis=1))
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return data, R, t

# Sample test
if __name__ == "__main__":
    # Random reference point cloud
    reference = np.random.rand(100, 2)

    # Create a transformed version of the reference point cloud
    true_rotation = np.array([[np.cos(np.pi/4), -np.sin(np.pi/4)], [np.sin(np.pi/4), np.cos(np.pi/4)]])
    true_translation = np.array([0.5, 0.3])
    data = np.dot(reference, true_rotation) + true_translation

    # Apply ICP
    aligned_data, estimated_rotation, estimated_translation = icp(data, reference)
    print("Estimated Rotation:\n", estimated_rotation)
    print("True Rotation:\n", true_rotation)
    print("------------------------")
    print("Estimated Translation:\n", estimated_translation)
    print("True transla2tion:\n", true_translation)

