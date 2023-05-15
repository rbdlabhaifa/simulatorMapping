import os
import json
import glob
import numpy as np
from sklearn.neighbors import NearestNeighbors

from numpy.linalg import norm


def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    # assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp_NN(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    # assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)
        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T, R, t = best_fit_transform(A, src[:m,:].T)

    return T, distances, i, R, t


def main():
    # Read the path to the CSV files from the JSON file
    with open(os.path.expanduser('../generalSettings.json'), 'r') as f:
        settings = json.load(f)
    path_to_csv_files = os.path.expanduser(settings['framesOutput'])
    path_to_cloud_points_orb_slam = os.path.expanduser(settings['mapInputDir'])
    path_to_cloud_points_orb_slam += "cloud1.csv"

    # Get a list of all CSV files in the directory
    files = glob.glob(os.path.join(path_to_csv_files, 'frame_*_orbs.csv'))

    # Initialize an empty list to store the point clouds
    point_clouds = []

    # Loop through all CSV files and plot the points
    for filename in files:
        point_cloud = np.genfromtxt(filename, delimiter=',', usecols=(0, 1, 2))
        point_clouds.append(point_cloud)

    # Load point clouds from CSV files
    point_cloud_A = np.vstack(point_clouds)
    point_cloud_B = np.genfromtxt(path_to_cloud_points_orb_slam, delimiter=',', usecols=(0, 1, 2))

    # Run the ICP algorithm
    transformation_mat, distances, iterations, rotation_mat, translation_vector = icp_NN(point_cloud_A, point_cloud_B)

    # Save the transformation matrix to a CSV file
    np.savetxt(path_to_csv_files + 'frames_lab_transformation_matrix.csv', transformation_mat, delimiter=',', fmt='%.6f')

    # Print a confirmation message
    print(f"Transformation matrix saved to '{path_to_csv_files + 'frames_lab_transformation_matrix.csv'}'")


if __name__ == "__main__":
    main()
