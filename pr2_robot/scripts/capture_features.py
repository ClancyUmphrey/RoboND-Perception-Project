#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from perception_training.training_helper import spawn_model
from perception_training.training_helper import delete_model
from perception_training.training_helper import initial_setup
from perception_training.training_helper import capture_sample
from perception_training.features import compute_color_histograms
from perception_training.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

import inputs as inp


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    # Models from each of the three worlds
    models = {}

    models[1] = [\
       'biscuits',
       'soap',
       'soap2',
       ]

    models[2] = [\
       'biscuits',
       'soap',
       'book',
       'soap2',
       'glue',
       ]

    models[3] = [\
       'sticky_notes',
       'book',
       'snacks',
       'biscuits',
       'eraser',
       'soap2',
       'soap',
       'glue',
       ]

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    for model_name in models[inp.WORLD]:
        spawn_model(model_name)
        
        for i in range(inp.N_CAPTURES):
            print 'Capturing %s, %d/%d'%(model_name, i+1, inp.N_CAPTURES)

            # Make five attempts to get a valid point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=True,
                                              nbins=inp.N_BINS)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals, nbins=inp.N_BINS)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()


    pickle.dump(labeled_features, open('training_set.sav', 'wb'))

