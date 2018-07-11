// STL
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <glob.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>          // for loading and saving PCD files
#include <pcl/io/ply_io.h>          // for loading and saving PLY files
#include <pcl/filters/voxel_grid.h> // for downsamling with voxel grid
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>  // for scaling of the point cloud
#include <pcl/common/centroid.h>    // for estimation of point cloud centroid and demeaning

// debugging:
#define output_differing_points

// switch behaviour:
bool allow_sensor_orientation_correction;
bool allow_recenter_clouds;
bool allow_scale_clouds;

/**
 * @brief get all files within a specified directory using wildcards
 * @note from: https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
 * @param pattern directory with wildcard like "./*"
 * @return vector of strings containing the full path of all files in the specified directory
 */
std::vector<std::string>
globVector(const std::string& pattern){
    glob_t glob_result;
    glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
    std::vector<std::string> files;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        files.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return files;
}


void
correctSensorOrientation(pcl::PointCloud<pcl::PointNormal>::Ptr &pointCloudPtr){
    // Set the sensor orientation and position to the identity if a inconsistent orientation
    // is given for the sensor. Usually caused by PCL bug, see:
    // https://github.com/PointCloudLibrary/pcl/pull/879
    // https://github.com/PointCloudLibrary/pcl/issues/626
    if(!pointCloudPtr->sensor_orientation_.isApprox(pointCloudPtr->sensor_orientation_.normalized(),
        0.0001f)){
        PCL_INFO("*** Sensor orientiation quaternion not normalized. Replacing by identity.\n");
        PCL_DEBUG("*** old orientation quaternion (xyzw): (%f, %f, %f, %f)\n",
                  (*pointCloudPtr).sensor_orientation_.x(),
                  (*pointCloudPtr).sensor_orientation_.y(),
                  (*pointCloudPtr).sensor_orientation_.z(),
                  (*pointCloudPtr).sensor_orientation_.w());

        (*pointCloudPtr).sensor_orientation_.setIdentity();
        (*pointCloudPtr).sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
    }
}


Eigen::Vector3f
recenterCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &pointCloudPtr){
   PCL_DEBUG("*** Recentering point cloud.\n");

   // calculate the centroid of the points
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*pointCloudPtr, centroid);

   // substract the centroid from the point cloud
   pcl::demeanPointCloud(*pointCloudPtr, centroid, *pointCloudPtr);

   // return centeroid as 3-component (xyz) vector
   return centroid.head<3>();
}


void
scaleCloud(float scaleFactor,
           pcl::PointCloud<pcl::PointNormal>::Ptr &pointCloudPtr){
   // exclude scaling values <= 0
   if(scaleFactor <= 0){
       PCL_ERROR("*** Scaling factor has to be > 0. Scaling skipped.\n");
       return;
   }

   // construct homogeneous transformation matrix for uniform scaling
   Eigen::Affine3f scaleTransform = Eigen::Affine3f::Identity();
   scaleTransform.scale(scaleFactor);

   // apply transformation matrix to point cloud and the sensor origin
   pcl::transformPointCloud(*pointCloudPtr, *pointCloudPtr, scaleTransform);
   pointCloudPtr->sensor_origin_ = scaleTransform * pointCloudPtr->sensor_origin_;

}


/**
 * @brief downsample_models_and_scenes
 * @param d_points_abs_
 */
void
downsample_models_and_scenes(float d_points_abs,
                             double scale_factor_model,
                             double scale_factor_scene,
                             bool recenter_model,
                             bool recenter_scene ){

    // ********************************************************************************************************************
    // *************************************************** DOWNSAMPLING ***************************************************
    // ********************************************************************************************************************

    // find models and scenes to downsample
    std::vector<std::string> models = globVector("/home/markus/ROS/datasets/geometric_primitives/models/with_normals/cuboid*");
    std::vector<std::string> scenes = globVector("/home/markus/ROS/datasets/geometric_primitives/scenes/scene*");

    unsigned int errorCounter = 0;


    // load models, downsample them and save them in a pcd file
    for (int i = 0; i < models.size(); i++ ){

        std::cout << "* downsampling the following model:" << std::endl;
        std::cout << "*** " << models[i] << std::endl;

        // load PLY file
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointNormal>);

        if (pcl::io::loadPLYFile<pcl::PointNormal> (models[i], *cloud) == -1) //* load the file
        {
            std::cout << "*** Couldn't read ply-file at " << models[i] << std::endl;
            errorCounter ++;
            continue;
        }
        std::cout << "*** Loaded "
                << cloud->width * cloud->height
                << " data points from ply-file (model)"
                << std::endl;

        if(allow_sensor_orientation_correction){
            // correct sensor orientation
            correctSensorOrientation(cloud);
        }
        if(allow_scale_clouds){
            // rescale the cloud if requested
            if(scale_factor_model != 1.0){
                scaleCloud(scale_factor_model, cloud);
                PCL_INFO("*** Model Point cloud rescaled.\n");
            }
        }
        if(allow_recenter_clouds){
            // recenter the cloud if requested
            if(recenter_model){
                recenterCloud(cloud);
                PCL_INFO("*** Model Point cloud recentered.\n");
            }
        }

        // downsample model
        pcl::VoxelGrid<pcl::PointNormal> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(d_points_abs, d_points_abs, d_points_abs);
        vg.setDownsampleAllData(true);
        vg.filter(*downsampled_cloud);

        // save downsampled model in pcd file
        std::size_t found = models[i].find_last_of("/");
        std::string modelName = models[i].substr(found+1);
        std::cout << "*** model: " << modelName << '\n';
        std::string fileName = "downsampled_model_cloud_" + modelName + ".pcd";

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
        // pcl version is older then 1.8.0, save the models in 1.7 folder
        pcl::io::savePCDFileASCII ("/home/markus/Documents/downsampled_models/PCL_1.7/" + fileName, *downsampled_cloud);
#else
        pcl::io::savePCDFileASCII ("/home/markus/Documents/downsampled_models/PCL_1.8/" + fileName, *downsampled_cloud);
#endif
    }


    // load scenes, downsample them and save them in a pcd file
    for (int i = 0; i < scenes.size(); i++ ){

        std::cout << "* downsampling the following scene:" << std::endl;
        std::cout << "*** " << scenes[i] << std::endl;

        // load pcd files
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointNormal>);

        if (pcl::io::loadPCDFile<pcl::PointNormal> (scenes[i], *cloud) == -1) //* load the file
        {
            std::cout << "*** Couldn't read pcd-file " << scenes[i] << std::endl;
            errorCounter ++;
            continue;
        }
        std::cout << "*** Loaded "
                << cloud->width * cloud->height
                << " data points from pcd-file (scene)"
                << std::endl;

        if(allow_sensor_orientation_correction){
            // correct sensor orientation
            correctSensorOrientation(cloud);
        }
        if(allow_scale_clouds){
            // rescale the cloud if requested
            if(scale_factor_scene != 1.0){
                scaleCloud(scale_factor_scene, cloud);
                PCL_INFO("*** Scene Point cloud rescaled.\n");
            }
        }
        if(allow_recenter_clouds){
            // recenter the cloud if requested
            if(recenter_scene){
                recenterCloud(cloud);
                PCL_INFO("*** Scene Point cloud recentered.\n");
            }
        }

        // downsample file
        pcl::VoxelGrid<pcl::PointNormal> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(d_points_abs, d_points_abs, d_points_abs);
        vg.setDownsampleAllData(true);
        vg.filter(*downsampled_cloud);

        // save downsampled scene file
        std::size_t found = scenes[i].find_last_of("/");
        std::string sceneName = scenes[i].substr(found+1);
        std::cout << "*** scene: " << sceneName << '\n';
        std::string fileName = "downsampled_model_cloud_" + sceneName + ".pcd";

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
        // pcl version is older then 1.8.0, save the scenes in 1.7 folder
        pcl::io::savePCDFileASCII ("/home/markus/Documents/downsampled_scenes/PCL_1.7/" + fileName, *downsampled_cloud);
#else
        pcl::io::savePCDFileASCII ("/home/markus/Documents/downsampled_scenes/PCL_1.8/" + fileName, *downsampled_cloud);
#endif
    }
    std::cout << errorCounter << " files could not be openend and therefore could not be downsampled." << std::endl;
}


/**
 * @brief compare
 * @param filesInDirectory_ONE
 * @param filesInDirectory_TWO
 */
void
compare(std::vector<std::string> &filesInDirectory_ONE,
        std::vector<std::string> &filesInDirectory_TWO){

    // ********************************************************************************************************************
    // **************************************************** COMPARISON ****************************************************
    // ********************************************************************************************************************

    if (filesInDirectory_ONE.size() != filesInDirectory_TWO.size()){
          std::cout << "There are not the same numbers of files to compare! Aborting..." << std::endl;
          return;
      }

    unsigned int differCounter = 0, equalCounter = 0, errorCounter = 0;

    for (int i = 0; i < filesInDirectory_ONE.size(); i++ ){

        std::cout << "* comparing the following pcd-files:" << std::endl;
        std::cout << "*** " << filesInDirectory_ONE[i] << " and\n    "
                  << filesInDirectory_TWO[i] << std::endl;

        // load pcd files
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ONE (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_TWO (new pcl::PointCloud<pcl::PointNormal>);

        if (pcl::io::loadPCDFile<pcl::PointNormal> (filesInDirectory_ONE[i], *cloud_ONE) == -1) //* load the file
        {
        std::cout << "*** Couldn't read pcd-file " << filesInDirectory_ONE[i] << std::endl;
        errorCounter ++;
        continue;
        }
        if (pcl::io::loadPCDFile<pcl::PointNormal> (filesInDirectory_TWO[i], *cloud_TWO) == -1) //* load the file
        {
        std::cout << "*** Couldn't read pcd-file " << filesInDirectory_TWO[i] << std::endl;
        errorCounter ++;
        continue;
        }
        std::cout << "*** Loaded "
                << cloud_ONE->width * cloud_ONE->height
                << " data points from pcd-file ONE"
                << std::endl;
        std::cout << "*** Loaded "
                << cloud_TWO->width * cloud_TWO->height
                << " data points from pcd-file TWO"
                << std::endl;

        // comparing both pcd files
        if(cloud_ONE->points.size() != cloud_TWO->points.size()){
              std::cout << "*** Attention! The pcd files do not contain an equal number of points! Aborting ..." << std::endl;
              differCounter ++;
              continue;
        }
        else{
          bool point_clouds_differ = false;
          unsigned int numberOfDifferentPoints  = 0;
          for (size_t j = 0; j < cloud_ONE->points.size(); j++){

            if ( !(cloud_ONE->points[j].x == cloud_TWO->points[j].x
                && cloud_ONE->points[j].y == cloud_TWO->points[j].y
                && cloud_ONE->points[j].z == cloud_TWO->points[j].z) ){
                    bool point_with_other_index_matches = false;
                    for(int k = 0; k < cloud_TWO->points.size(); k++){
                            if (  (cloud_ONE->points[j].x == cloud_TWO->points[k].x
                                && cloud_ONE->points[j].y == cloud_TWO->points[k].y
                                && cloud_ONE->points[j].z == cloud_TWO->points[k].z) ){

//                                    #ifdef output_differing_points
//                                    std::cout << "At point "<< j << " of the first cloud, the point matches point "<< k <<" of the second cloud:\n"
//                                          << "    " << cloud_ONE->points[j].x << " - " << cloud_TWO->points[k].x << "\n"
//                                          << "    " << cloud_ONE->points[j].y << " - " << cloud_TWO->points[k].y << "\n"
//                                          << "    " << cloud_ONE->points[j].z << " - " << cloud_TWO->points[k].x << std::endl;
//                                    #endif
                                    point_with_other_index_matches = true;
                                    break;
                                }
//                            #ifdef output_differing_points
//                            std::cout << "comparing point "<< j << " of the first cloud, with point "<< k <<" of the second cloud:\n"
//                                  << "    " << cloud_ONE->points[j].x << " - " << cloud_TWO->points[k].x << "\n"
//                                  << "    " << cloud_ONE->points[j].y << " - " << cloud_TWO->points[k].y << "\n"
//                                  << "    " << cloud_ONE->points[j].z << " - " << cloud_TWO->points[k].x << std::endl;
//                            #endif
                        }

                    #ifdef output_differing_points
                    std::cout << "At point "<< j << ", the point clouds are not the same:\n"
                          << "    " << cloud_ONE->points[j].x << " - " << cloud_TWO->points[j].x << "\n"
                          << "    " << cloud_ONE->points[j].y << " - " << cloud_TWO->points[j].y << "\n"
                          << "    " << cloud_ONE->points[j].z << " - " << cloud_TWO->points[j].x << std::endl;
                    #endif

                    if(!point_with_other_index_matches){
                            numberOfDifferentPoints ++;
                            point_clouds_differ = true;
                        }
            }
          }
          if (!point_clouds_differ){
                  std::cout << "*** :) the compared point clouds are the same!" << std::endl;
                  equalCounter ++;
              }
          else{
                  std::cout << "*** :( the compared point clouds are NOT the same! " << numberOfDifferentPoints
                            << " of " << cloud_ONE->points.size() << " points differ!" << std::endl;
                  differCounter ++;
              }
        }
    } // i

    std::cout << "** " << equalCounter << " comparisons resulted in EQUAL point clouds, "
                     << differCounter << " comparisons resulted in UNequal point clouds. There were "
                     << errorCounter << " load-errors!" << std::endl;
}


/**
 * @brief compare_models_and_scenes
 */
void
compare_models_and_scenes(){

    // compare scenes
    std::cout << "comparing scenes..." << std::endl;
    std::vector<std::string> filesInDirectory_ONE = globVector("/home/markus/Documents/downsampled_scenes/PCL_1.7/*");
    std::vector<std::string> filesInDirectory_TWO = globVector("/home/markus/Documents/downsampled_scenes/PCL_1.8/*");

    compare(filesInDirectory_ONE, filesInDirectory_TWO);

    filesInDirectory_ONE.clear();
    filesInDirectory_TWO.clear();

    // compare models
    std::cout << "comparing models..." << std::endl;
    filesInDirectory_ONE = globVector("/home/markus/Documents/downsampled_models/PCL_1.7/*");
    filesInDirectory_TWO = globVector("/home/markus/Documents/downsampled_models/PCL_1.8/*");

    compare(filesInDirectory_ONE, filesInDirectory_TWO);
}


int
main (int argc, char* argv[])
{
    if (argc < 2){
        std::cout << "argument is missing, choose one from '-downsample' and '-compare'" << std::endl;
        return (-1);
    }
    std::string doDownsample = "-downsample";
    std::string doComparison = "-compare";

    if (argv[1] == doDownsample ){

        if (argc != 5){
            std::cout << "downsample arguments are missing, you must choose true (1) or false (0) for all of the following three options\n"
                         "-'allow_sensor_orientation_correction' and \n-'allow_scale_clouds'"
                         "and \n-'allow_recenter_clouds'" << std::endl;
            return (-1);
        }
        allow_sensor_orientation_correction = argv[2];
        allow_scale_clouds = argv[3];
        allow_recenter_clouds = argv[4];

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
        // pcl version is older then 1.8.0, save the models in 1.7 folder
        std::cout << "using PCL_1.7, do you want to continue downsampling?" << std::endl;
#else
        std::cout << "using PCL_1.8, do you want to continue downsampling? Press 'y' if so, other keys abort the program." << std::endl;
#endif
        std::string userInput;
        std::cin >> userInput;
        if(userInput == "y") std::cout << "continuing..." << std::endl;
        else {
            std::cout << "aborting..." << std::endl;
            return (-1);
        }

        // set d_points_abs, the downsampling distance:
        float d_points_abs = 0.00975309;  // value up to the decimal point that both systems, mine and Xavers, agree on
        double scale_factor_model = 0.001;
        double scale_factor_scene = 1.0;
        bool recenter_model = true;
        bool recenter_scene = false;
        downsample_models_and_scenes(d_points_abs, scale_factor_model, scale_factor_scene, recenter_model, recenter_scene);
        return (0);
    }
    if (argv[1] == doComparison ){
        compare_models_and_scenes();
        return (0);
    }
    else{
        std::cout << "unknown argument, choose one from '-downsample' and '-compare'" << std::endl;
        return (-1);
    }

    return (0);
}
