# voxelGrid_filter_PCL_1.8

use like this:

1. to apply voxel grid filter to a point cloud (its path is to be specified in the c++ source-file):

	./downsampling_compare -downsample 0 0 0

2. as a next step you'd have to do step 1. with the code from 'voxelGrid_filter_PCL_1.7' repository.

then you have one downsampled version of the point cloud filtered with voxelGrid filter of PCL 1.7 and another one filtered with voxelGrid filter of PCL 1.8

3. now you can compare the two downsampled point clouds with

	./downsampling_compare -compare

Expected result: The filter should produce different results for both PCL versions.
