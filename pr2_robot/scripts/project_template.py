#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
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

# Helper function to calculate euclidian distance error of centroid estimate and truth
def calc_centroid_err(est, true):
    return np.sqrt((est[0]-true[0])**2 + (est[1]-true[1])**2 + (est[2]-true[2])**2)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

  ### Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()
    # Choose a voxel (also known as leaf) size (resolution of downsampling)
    LEAF_SIZE = 0.005   
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    voxel_cloud = cloud_filtered

  ### PassThrough Filter
    # Create a PassThrough filter object.
    passthroughz = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthroughz.set_filter_field_name(filter_axis)
    z_axis_min = 0.605
    z_axis_max = 0.9
    passthroughz.set_filter_limits(z_axis_min, z_axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthroughz.filter()

    passthroughy = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthroughy.set_filter_field_name(filter_axis)
    y_axis_min = -0.50
    y_axis_max = 0.50
    passthroughy.set_filter_limits(y_axis_min, y_axis_max)
    cloud_filtered = passthroughy.filter()
  
    passthrough_cloud = cloud_filtered

  ### Outlier Removal Filter
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(25)
    # Set threshold scale factor
    x = 0.3
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()


  ### RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

  ### Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)

  #### Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

  ### Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    #     print("i, num particles for each obj: ",i)
    # print("-------------------------------------")
    
    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_filtered = pcl_to_ros(cloud_filtered)
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_cloud_filt_pub.publish(ros_cloud_filtered)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_clusters_pub.publish(ros_cluster_cloud)
    voxel_pub.publish(pcl_to_ros(voxel_cloud))
    passthrough_pub.publish(pcl_to_ros(passthrough_cloud))

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        # pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = extracted_outliers.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        sample_cloud = pcl_to_ros(ros_cluster)
        # Extract histogram features
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # labeled_features.append([feature, model_name])

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # WORLD_NUM = 2 # Make sure to be consistent
    WORLD_NUM = rospy.get_param('/test_scene_num')
    
    # Initialize variables
    labels = []
    centroids = [] # to be list of tuples (x,y,z)
    
    # ROS server message
    test_scene_num = Int32()
    test_scene_num.data = WORLD_NUM
    object_name = String()
    arm_name = String()
    pick_pose = Pose() 
    dict_list = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    obj_loc = rospy.get_param('/object_locations')

    obj_loc_names = []
    for obj in obj_loc:
        obj_loc_names.append(obj['name'])


    left_box_pose = Pose()
    left_box_pose.position.x = dropbox_param[0]['position'][0]
    left_box_pose.position.y = dropbox_param[0]['position'][1]
    left_box_pose.position.z = dropbox_param[0]['position'][2]

    right_box_pose = Pose()
    right_box_pose.position.x = dropbox_param[1]['position'][0]
    right_box_pose.position.y = dropbox_param[1]['position'][1]
    right_box_pose.position.z = dropbox_param[1]['position'][2]

    # TODO: Parse parameters into individual variables
    obj_names = []
    obj_groups = []
    for obj in object_list_param:
        obj_names.append(obj['name'])
        obj_groups.append(obj['group'])

    # Extract centroids for all classified objects
    for obj in object_list:
        labels.append(obj.label)
        points_arr = obj.cloud.to_array()
        # points_arr = ros_to_pcl(obj.cloud).to_array()
        mea = np.mean(points_arr, axis=0)[:3]
        mea = [np.asscalar(x) for x in mea]
        centroids.append(mea)

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list
    for obj in object_list_param:
        # Check if found any of desired object
        indices = [i for i,x in enumerate(labels) if x == obj['name']]
        if(len(indices)==0):
            rospy.loginfo("Couldn't find %s"%obj['name'])
            continue
        # Multiple occurances
        elif(len(indices) > 1):
            # Take the one with the most points in the cloud
            lens = [do.cloud.size for do in [object_list[i] for i in indices]]
            ind = indices[np.argmax(lens)]
            # rospy.loginfo("Multiple %s found, of cloud sizes ",lens)
        else:
            ind = indices[0]

        # Store object name
        object_name.data = obj['name']
        
        # Get centroid
        pick_pose.position.x = centroids[ind][0]
        pick_pose.position.y = centroids[ind][1]
        pick_pose.position.z = centroids[ind][2]

        ## Compare to ground truth
        true_centroid = obj_loc[obj_loc_names.index(obj['name'])]['centroid'] # Assumes same ordering of objects in yaml files
        cent_err = calc_centroid_err(centroids[ind], true_centroid)
        rospy.loginfo("Centroid Error for %s (m): %.3f"%(obj['name'],cent_err))
        if(cent_err > 0.20):
            rospy.loginfo("  CENTROID ERROR")

        # Create 'place_pose' for the object
        # Assign the arm to be used for pick_place
        if(obj['group'] == 'green'):
            arm_name.data = 'right'
            place_pose = right_box_pose
        else:
            arm_name.data = 'left'
            place_pose = left_box_pose

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)


        # # Wait for 'pick_place_routine' service to come up
        # rospy.wait_for_service('pick_place_routine')

        # try:
        #     pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        #     # TODO: Insert your message variables to be sent as a service request
        #     resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

        #     print ("Response: ",resp.success)

        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    yaml_filename = "output_{}.yaml".format(WORLD_NUM)
    send_to_yaml(yaml_filename, dict_list)
    rospy.loginfo("Saved yaml output as %s"%yaml_filename)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    
    # Create Subscribers
    # pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)    
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1) 

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    pcl_cloud_filt_pub = rospy.Publisher("/pcl_cloud_filt", PointCloud2, queue_size=1)
    pcl_cloud_filt_pub = rospy.Publisher("/pcl_cloud_filt", PointCloud2, queue_size=1)
    voxel_pub = rospy.Publisher("/voxel", PointCloud2, queue_size=1)
    passthrough_pub = rospy.Publisher("/passthrough", PointCloud2, queue_size=1)

    # Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    # model = pickle.load(open('model.sav', 'rb'))
    # model = pickle.load(open('model_project_15shots_32bins_hsv.sav', 'rb'))
    model = pickle.load(open('model_project_100shots_16bins_hsv.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()