# voxelGrid_filter_PCL_1.8

Installation:

1. cmake CMakeLists.txt
2. make
3. DONE

Use like this:

1. to apply voxel grid filter to a point cloud, specify its load path and saving path:
   you should use the point clouds provided in this repository. the downsampling distance is coded in the .cpp file, i.e you would have to manually change that value. it is called 'd_points_abs'.

	./downsampling_compare -downsample /path/to/point/cloud/ /saving/path/for/downsampled/cloud/

2. as a next step you'd have to do step 1. with the code from 'voxelGrid_filter_PCL_1.7' repository.

then you have one downsampled version of the point cloud filtered with voxelGrid filter of PCL 1.7 and another one filtered with voxelGrid filter of PCL 1.8

3. now you can compare the two downsampled point clouds with (use same paths as in step 1)

	./downsampling_compare -compare /path/to/point/cloud/ /saving/path/for/downsampled/cloud/

Expected result: The filter should produce different results for both PCL versions.
