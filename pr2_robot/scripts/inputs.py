from math import sqrt

# Training Set
#-----------------------------------------------------------------------------------------
N_CAPTURES = 200   # number of captures per model
N_BINS     = 64    # number of histogram bins per feature component
WORLD      = 3     # which world models to capture  

# Statistical Outlier Filtering
#-----------------------------------------------------------------------------------------
K          = 5     # number of neighboring points to analyze
X          = 0.01  # threshold scale factor (outlier: distance > mean_distance+X*std_dev)

# Voxel Grid Downsampling
#-----------------------------------------------------------------------------------------
LEAF_SIZE  = 0.007 # voxel (or leaf) size in meters

# PassThrough Filter
#-----------------------------------------------------------------------------------------
Z_AX_MIN   = 0.605 # below table top 
Z_AX_MAX   = 0.850 # above tallest object (snacks) 
X_AX_MIN   = 0.310 # before table
X_AX_MAX   = 0.868 # after table

# RANSAC Plane Segmentation
#-----------------------------------------------------------------------------------------
MAX_DIST   = 0.01  # max distance for a point to be considered fitting the model

# Euclidean Clustering
#-----------------------------------------------------------------------------------------
# Distance threshold is set to voxel diagonal.
# Cluster size thresholds lead with an object surface area term.
CLUSTER_TOL      = sqrt(3) * LEAF_SIZE     # distance threshold
CLUSTER_SIZE_MIN = 0.004   * LEAF_SIZE**-2 # minimum number of cluster points
CLUSTER_SIZE_MAX = 0.100   * LEAF_SIZE**-2 # maximum number of cluster points

