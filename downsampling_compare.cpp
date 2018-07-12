// STL
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <regex> // for checking file extensions

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
//#define output_differing_points
#define load_model_PLY


/**
 * @brief Check whether a given file path has a certain file extension
 * @param path file path to check
 * @param ending file ending without "." (e.g. "ply" for .ply-files)
 * @return true if path has the ending, else false
 */
static bool checkExtension(std::string path, std::string ending){
    std::regex e("(.*)(\\." + ending + ")");
    return std::regex_match(path, e,std::regex_constants::ECMAScript);
}


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


/**
 * @brief downsample model
 * @param d_points_abs the leaf size for the filter
 */
void
downsample(float d_points_abs, std::string loadPath, std::string savePath){

    // ********************************************************************************************************************
    // *************************************************** DOWNSAMPLING ***************************************************
    // ********************************************************************************************************************

    // find models that match the specified pattern to downsample. use * as wildcard
    std::vector<std::string> models = globVector(loadPath + "*");

    unsigned int errorCounter = 0;

    // load models, downsample them and save them in a pcd file
    for (int i = 0; i < models.size(); i++ ){

        std::cout << "* downsampling the following model:" << std::endl;
        std::cout << "*** " << models[i] << std::endl;

        // load file
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointNormal>);

        // load file according to file extension
        bool load_success = false;
        if(checkExtension(models[i],"pcd")){
            // load .pcd file
            load_success = !pcl::io::loadPCDFile(models[i], *cloud); // returns 0 at success
        }
        else if(checkExtension(models[i],"ply")){
            // load .ply file
            load_success = !pcl::io::loadPLYFile(models[i], *cloud); // returns 0 at success
        }
        else{
            // file extension is unknown
            PCL_ERROR("File format not supported for loading.\n");
            load_success = false;
        }
        if(!load_success){
            std::cout << "*** Couldn't read file at " << models[i] << std::endl;
            errorCounter ++;
            continue;
        }
        std::cout << "*** Loaded "
                << cloud->width * cloud->height
                << " data points from file (model)"
                << std::endl;

        // handle models and scenes differently
        std::size_t found = models[i].find("models");
        if (found!=std::string::npos){
            d_points_abs = d_points_abs * 1000; // models are in mm, scenes in m, ergo models d_points_abs need to be multiplied with 1000
        }

        // downsample model
        pcl::VoxelGrid<pcl::PointNormal> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(d_points_abs, d_points_abs, d_points_abs);
        vg.setDownsampleAllData(true);
        vg.filter(*downsampled_cloud);

        // save downsampled model in pcd file
        found = models[i].find_last_of("/");
        std::string modelName = models[i].substr(found+1);
        std::cout << "*** model: " << modelName << '\n';

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
        // pcl version is older then 1.8.0, save accordingly
        std::string fileName = "downsampled_model_cloud_PCL_1.7.2_" + modelName + ".pcd";
        pcl::io::savePCDFileASCII (savePath + fileName, *downsampled_cloud);
#else
        std::string fileName = "downsampled_model_cloud_PCL_1.8.1_" + modelName + ".pcd";
        pcl::io::savePCDFileASCII (savePath + fileName, *downsampled_cloud);
#endif
    }

    std::cout << errorCounter << " files could not be openend and therefore could not be downsampled." << std::endl;
}


/**
 * @brief compare all point coordinates of two models
 * @param loadPath equals the savePath of downsample()
 */
void
compare(std::string loadPath){

    // ********************************************************************************************************************
    // **************************************************** COMPARISON ****************************************************
    // ********************************************************************************************************************

    std::cout << "comparing models..." << std::endl;
    std::vector<std::string> filesInDirectory_ONE = globVector(loadPath + "downsampled_model_cloud_PCL_1.7.2_*");
    std::vector<std::string> filesInDirectory_TWO = globVector(loadPath + "downsampled_model_cloud_PCL_1.8.1_*");

    if (filesInDirectory_ONE.size() != filesInDirectory_TWO.size()){
          std::cout << "There are not the same numbers of files to compare! Aborting..." << std::endl;
          return;
      }
    if(filesInDirectory_ONE.empty() || filesInDirectory_TWO.empty()){
        PCL_ERROR("No models to compare where found! Check file names! Aborting...");
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

                                    point_with_other_index_matches = true;
                                    break;
                                }
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



int
main (int argc, char* argv[])
{
    if (argc < 4){
        std::cout << "argument is missing, choose one from '-downsample' and '-compare' AND ADD load and save paths" << std::endl;
        return (-1);
    }
    std::string doDownsample = "-downsample";
    std::string doComparison = "-compare";

    if (argv[1] == doDownsample ){

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
        // pcl version is older then 1.8.0, save the models in 1.7 folder
        std::cout << "using PCL_1.x with x < 8, do you want to continue downsampling? Press 'y' if so, other keys abort the program." << std::endl;
#else
        std::cout << "using PCL_1.8 or newer, do you want to continue downsampling? Press 'y' if so, other keys abort the program." << std::endl;
#endif
        std::string userInput;
        std::cin >> userInput;
        if(userInput == "y") std::cout << "continuing..." << std::endl;
        else {
            std::cout << "aborting..." << std::endl;
            return (-1);
        }

        // set d_points_abs, the downsampling distance
        float d_points_abs = 0.00975309;  // value for cuboid model

        // do downsampling
        downsample(d_points_abs, argv[2], argv[3]);
        return (0);
    }
    if (argv[1] == doComparison ){
        compare(argv[3]);
        return (0);
    }
    else{
        std::cout << "unknown argument, choose one from '-downsample' and '-compare' AND ADD load and save paths" << std::endl;
        return (-1);
    }

    return (0);
}
