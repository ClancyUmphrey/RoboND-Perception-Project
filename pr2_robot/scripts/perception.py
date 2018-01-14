#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from perception_training.features import compute_color_histograms
from perception_training.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import os
import rospkg
import inputs as inp

# Control Flags
VERBOSE = False

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    #----------------------------------------------------------------
    cloud_filtered = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    #----------------------------------------------------------------
    # Much like the other filters, we start by creating a filter object: 
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(inp.K)
    
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be
    # considered outlier
    outlier_filter.set_std_dev_mul_thresh(inp.X)
    
    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
    
    # Publish Outlier-Filtered Cloud
    pcl_rm_outlier_pub.publish(pcl_to_ros(cloud_filtered))

    # TODO: Voxel Grid Downsampling
    #----------------------------------------------------------------
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()
    
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(inp.LEAF_SIZE, inp.LEAF_SIZE, inp.LEAF_SIZE)
    
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    #----------------------------------------------------------------
    def passthrough_filter(cloud, axis, ax_min, ax_max, limit_negative=False):
        '''
        Returns a passthrough filtered point cloud.
        '''
        # Create a PassThrough filter object
        passthrough = cloud.make_passthrough_filter()
        # Assign axis and range to the passthrough filter object
        passthrough.set_filter_field_name(axis)
        passthrough.set_filter_limits(ax_min, ax_max)
        # Use the filter function to obtain the resultant point cloud
        return passthrough.filter()
        
    # Remove points below the table and above the tallest object (snacks)
    cloud_filtered = passthrough_filter(cloud_filtered, 'z', inp.Z_AX_MIN, inp.Z_AX_MAX)
    # Remove points in front of the table and behind the table
    # (upper limit of boxes ~ 0.310; back of table ~ 0.868)
    cloud_filtered = passthrough_filter(cloud_filtered, 'x', inp.X_AX_MIN, inp.X_AX_MAX)
    # NOTE: Not compatible with collision avoidance of side boxes

    
    # TODO: RANSAC Plane Segmentation
    #----------------------------------------------------------------
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    
    # Max distance for a point to be considered fitting the model
    seg.set_distance_threshold(inp.MAX_DIST)
    
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    #----------------------------------------------------------------
    cloud_table   = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    #----------------------------------------------------------------
    white_cloud = XYZRGB_to_XYZ(cloud_objects) 
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(inp.CLUSTER_TOL)
    ec.set_MinClusterSize(inp.CLUSTER_SIZE_MIN)
    ec.set_MaxClusterSize(inp.CLUSTER_SIZE_MAX)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #----------------------------------------------------------------
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    
    color_cluster_point_list = []
    
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    
    # Create new cloud containing all clusters, each with unique color
    cloud_cluster = pcl.PointCloud_PointXYZRGB()
    cloud_cluster.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages and publish
    #----------------------------------------------------------------
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_table_pub.publish  (pcl_to_ros(cloud_table  ))
    pcl_cluster_pub.publish(pcl_to_ros(cloud_cluster))

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    #----------------------------------------------------------------
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        if VERBOSE: print 'index ',index,' npts ',len(pts_list)

        # Grab the points for the cluster
        #------------------------------------------------------------
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        #------------------------------------------------------------
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True, nbins=inp.N_BINS)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals, nbins=inp.N_BINS)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        #------------------------------------------------------------
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list.
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        #------------------------------------------------------------
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects
        #------------------------------------------------------------
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    #----------------------------------------------------------------
    detected_objects_pub.publish(detected_objects)

    # Call pr2_mover() to write yaml file of identified picklist objects
    # (Actually moving the objects is not currently implemented)
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# Function to load parameters and output yaml file of identified picklist objects
# Also the place to implement PickPlace service request
def pr2_mover(detected_objects):

    # TODO: Get/Read parameters
    #----------------------------------------------------------------
    object_list_param = rospy.get_param('/object_list')
    dropbox_param     = rospy.get_param('/dropbox')
    test_scene_num    = Int32(data=rospy.get_param('/test_scene_num'))

    # TODO: Parse parameters into individual variables
    #----------------------------------------------------------------
    box_position = {}
    box_position['left' ] = dropbox_param[0]['position']
    box_position['right'] = dropbox_param[1]['position']
    
    # TODO: Loop through the pick list
    #----------------------------------------------------------------
    yaml_dict_list = []
    for pick_object in object_list_param:

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        #------------------------------------------------------------
        for detected_object in detected_objects:
            if detected_object.label == pick_object['name']:
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                centroid = [float(xyz) for xyz in np.mean(points_arr, axis=0)[:3]]
                break
        else:
            print 'pick_object %s not among detected_objects' % pick_object['name']
            continue # Move on to next pick_object

        # Create the 'object_name' for the object
        #------------------------------------------------------------
        object_name = String(data=pick_object['name'])

        # TODO: Assign the arm to be used for pick_place
        #------------------------------------------------------------
        arm      = 'left' if pick_object['group'] == 'red' else 'right'
        arm_name = String(data=arm)

        # Create 'pick_pose' for the object
        #------------------------------------------------------------
        pick_pose = Pose()
        pick_pose.position.x = centroid[0]
        pick_pose.position.y = centroid[1]
        pick_pose.position.z = centroid[2]

        # TODO: Create 'place_pose' for the object
        #------------------------------------------------------------
        place_pose = Pose()
        place_pose.position.x = box_position[arm][0]
        place_pose.position.y = box_position[arm][1]
        place_pose.position.z = box_position[arm][2]


        # TODO: Create a list of dictionaries for later output to yaml format
        #------------------------------------------------------------
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose,
                                   place_pose)
        yaml_dict_list.append(yaml_dict)

    # TODO: Output your request parameters into output yaml file
    #----------------------------------------------------------------
    # Get directory that the pr2_robot package resides in
    pkg_dir        = os.path.dirname(rospkg.RosPack().get_path('pr2_robot'))
    # Set Output directory path to place output yaml files in
    outdir         = os.path.join(pkg_dir,'output')
    # Set Output yaml file path
    yaml_file      = 'output_%d.yaml' % test_scene_num.data
    yaml_file_path = os.path.join(outdir, yaml_file)
    # Create outdir if it does not already exist
    if not os.path.isdir(outdir): os.mkdir(outdir)

    send_to_yaml(yaml_file_path, yaml_dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    #----------------------------------------------------------------
    rospy.init_node('perception', anonymous=True)

    # TODO: Create Subscribers
    #----------------------------------------------------------------
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    #----------------------------------------------------------------
    pcl_rm_outlier_pub = rospy.Publisher("/pcl_rm_outlier", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    #----------------------------------------------------------------
    try:
        model = pickle.load(open('model.sav', 'rb'))
        clf = model['classifier']
        encoder = LabelEncoder()
        encoder.classes_ = model['classes']
        scaler = model['scaler']
    except Exception as e:
        print 'Unable to load model.sav: ', e

    # Initialize color_list
    #----------------------------------------------------------------
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    #----------------------------------------------------------------
    while not rospy.is_shutdown():
        rospy.spin()
