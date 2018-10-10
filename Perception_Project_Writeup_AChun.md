## Project: Perception Pick & Place
### Austin Chun
### Oct 2018

---
[//]: # (Image References)
[Input]: ./writeup_images/Input.png
[Voxel]: ./writeup_images/Voxel.png
[Passthrough]: ./writeup_images/Passthrough.png
[Outlier]: ./writeup_images/Outlier.png
[Table]: ./writeup_images/Table.png
[Objects]: ./writeup_images/Objects.png


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
As instructed in Exercise 1, the filtering pipeline is setup as follows:

1. Read in Point Cloud (Convert from ROS to PCL)
2. Voxel Grid Downsampling
3. PassThrough Filter (used z and y)
4. Outlier Removal Filter
5. RANSAC Plane Segmentation

For the Voxel Grid Downsampling (lines 60-68), I used a LEAF_SIZE = 0.005. I decided to use two passthrough filters (lines 70-88), one for Z as normal, but also one for y (left-right view from camera) to eliminate corners of the bins. Then the outlier removal filter (lines 90-100) gets rid of noise (looked at 25 neighbors, and threshold scale factor as 0.3). Lastly, RANSAC Plane Segmentation (lines 103-117) separates the table and the objects on the table.

![Input][Input]
![Voxel][Voxel]
![Passthrough][Passthrough]
![Outlier][Outlier]
![Table][Table]
![Objects][Objects]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



