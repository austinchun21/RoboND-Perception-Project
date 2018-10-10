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
[Clusters]: ./writeup_images/Clusters.png
[Confusion]: ./writeup_images/Confusion.png
[Detection1]: ./writeup_images/Detection1.png
[Detection2]: ./writeup_images/Detection2.png
[Detection3]: ./writeup_images/Detection3.png

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
<p style="text-align: center;"> **Input Cloud** </p>
![Input][Input]
<p style="text-align: center;"> **Voxel Downsampling** </p>
![Voxel][Voxel]
<p style="text-align: center;"> **PassThrough Filter (Z and Y)** </p>
![Passthrough][Passthrough]
<p style="text-align: center;"> **Outlier Removal Filter** </p>
![Outlier][Outlier]
<p style="text-align: center;"> **Extracted Table (RANSAC)** </p>
![Table][Table]
<p style="text-align: center;"> **Extracted Objects (RANSAC)** </p>
![Objects][Objects]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
With the objects extracted from the scene, the objects point cloud can be clustered, in this case using Euclidian clustering. I used a ClusterTolerance = 0.05, with ClusterSize between 50 and 3000. Below is an image of the clustering for World3.

![Clusters][Clusters]


#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
For the feature extraction, I filled in `computer_color_histograms()` and `computer_normal_histograms()` in `features.py` using bins=16 (I decreased the bin size to speed up the training data capturing process). I also increased the number of random positions to 100 positions for each object (`capture_features.py` line 57). As for training the SVM, I used the provided code, but didn't change any parameters, since the default worked well. The resulting confusion matrices are shown below.

![Confusion][Confusion]

The object prediction code is added to `project_template.py`. The resulting object detection is shown below for Worlds 1, 2, and 3.

![Detection1][Detection1]
![Detection2][Detection2]
![Detection3][Detection3]

The object detection works for 3/3 objects in World 1, 4/5 objects in World 2, and 7/8 objects in world 3. Consistently, the glue is mislabeled as biscuits... which is confusing, but it works well enough.

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

In `project_template.py`, the function `pr2_mover()` (lines 219 - 335) read in the pick-list, and construct the appropriate request output. Ad mentioned in the previous section, the only issue with the object detection is with glue (it gets misclassified as biscuits in World 2 and 3).

Some other changes I made include adding a centroid error check using ground truth centroid positions (extracted from the `.world` files, and formatted into `ob_locations_*.yaml` under `pr2_robot/config/`). Also, if there are multiple classifications for a single object, the code returns the object that has the most points in its point cloud (a simple heuristic to try avoid misclassification due to small cluster sizes).

It would be great to complete the cycle, and implement the pick and place actions, but for now I need to get onto Project 4. Other improvements could include speed performance (maybe smaller Voxel sizes, or only execute expensive calculations periodically), or improving the model by editing the training data generation to produce 'occluded' versions of the objects, to train the SVM on partial images. 
