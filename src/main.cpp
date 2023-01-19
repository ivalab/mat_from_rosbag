#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <boost/filesystem.hpp>

// Matlab header, this is needed to save mat files
// Note that we use the FindMatlab.cmake to get this
// #include "mat.h"

using namespace std;

/*
bool writeToMat(const vector<double> & dataBuf, const string & pathMat, const string & varName) {
  
    // Create the matlab mat file
    MATFile *pmat = matOpen(pathMat.c_str(), "w");
    if (pmat == NULL) {
        ROS_ERROR("Error could not create the mat file");
        return(EXIT_FAILURE);
    }
    
    int N = dataBuf.size()/8;
    printf("In total %d records are about to be saved\n", N);
    
    mxArray *pa1 = mxCreateDoubleMatrix(N,8,mxREAL);
    if (pa1 == NULL) {
        printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
        printf("Unable to create mxArray.\n");
        return false;
    }
    // Correctly copy data over (column-wise)
    double* pt1 = mxGetPr(pa1);
     for(size_t i=0; i<dataBuf.size(); i+=8) {
         pt1[i/8] = dataBuf.at(i);
         pt1[i + N] = dataBuf.at(i+1);
         pt1[i + N*2] = dataBuf.at(i+2);
         pt1[i + N*3] = dataBuf.at(i+3);
         pt1[i + N*4] = dataBuf.at(i+4);
         pt1[i + N*5] = dataBuf.at(i+5);
         pt1[i + N*6] = dataBuf.at(i+6);
         pt1[i + N*7] = dataBuf.at(i+7);
     }
    // Add it to the matlab mat file
    int status = matPutVariable(pmat, varName.c_str(), pa1);
    if(status != 0) {
        printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
        return false;
    }
    
    // Cleanup
    mxDestroyArray(pa1);
    ROS_INFO("Done writing data to mat");

    // Close the mat file
    if (matClose(pmat) != 0) {
        ROS_ERROR("Error closing the mat file");
        return false;
    }
}
*/

bool writeToText(const vector<double> & dataBuf, const string & pathText) {
  
    int N = dataBuf.size()/8;
    printf("In total %d records are about to be saved\n", N);
    
    ofstream fLog;
    fLog.open(pathText.c_str());
    fLog << fixed;
    for(size_t i=0; i<N; i++)
    {
        fLog << setprecision(6)
                      << dataBuf.at(0+i*8) << " "
                      << dataBuf.at(1+i*8) << " "
		      << dataBuf.at(2+i*8) << " "
		      << dataBuf.at(3+i*8) << " "
		      << dataBuf.at(4+i*8) << " "
		      << dataBuf.at(5+i*8) << " "
		      << dataBuf.at(6+i*8) << " "
		      << dataBuf.at(7+i*8) << std::endl;
    }
    fLog.close();
    
    return true;
}


int main(int argc, char **argv) {

    // Debug message
    ROS_INFO("Starting up");

    // Check if there is a path to a dataset
    if(argc < 5) {
        ROS_ERROR("Error please specify a rosbag file");
        ROS_ERROR("Command Example: rosrun mat_from_rosbag mat_from_rosbag <rosbag> <topic_odom_1> <topic_odom_2> <topic_pose>");
        return EXIT_FAILURE;
    }

    // Startup this node
    ros::init(argc, argv, "mat_from_rosbag");

    // Parse the input
    string pathBag = argv[1];
    string odomTopic_1 = argv[2];
    string odomTopic_2 = argv[3];
    string pathTopic = argv[4];

    // Get path
    boost::filesystem::path p(pathBag);
    string pathParent = p.parent_path().string();

    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);


    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view(bag);

    // Debug
    ROS_INFO("BAG Path is: %s", pathBag.c_str());
    ROS_INFO("Reading in rosbag file...");

    // Our data vector
    vector<double> dataOdom_1 = vector<double>();
    vector<double> dataOdom_2 = vector<double>();
    vector<double> dataPath = vector<double>();

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
      
      if (m.getTopic() == odomTopic_1) {
	nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
	if (s != NULL) {
            // dataOdom_1.push_back(m.getTime().toSec());
            dataOdom_1.push_back(s->header.stamp.toSec());
            dataOdom_1.push_back(s->pose.pose.position.x);
            dataOdom_1.push_back(s->pose.pose.position.y);
            dataOdom_1.push_back(s->pose.pose.position.z);
            dataOdom_1.push_back(s->pose.pose.orientation.x);
            dataOdom_1.push_back(s->pose.pose.orientation.y);
            dataOdom_1.push_back(s->pose.pose.orientation.z);
            dataOdom_1.push_back(s->pose.pose.orientation.w);
	}
      }
      else if (m.getTopic() == odomTopic_2) {
	nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
	if (s != NULL) {
            dataOdom_2.push_back(s->header.stamp.toSec());
            // dataOdom_2.push_back(m.getTime().toSec());
            dataOdom_2.push_back(s->pose.pose.position.x);
            dataOdom_2.push_back(s->pose.pose.position.y);
            dataOdom_2.push_back(s->pose.pose.position.z);
            dataOdom_2.push_back(s->pose.pose.orientation.x);
            dataOdom_2.push_back(s->pose.pose.orientation.y);
            dataOdom_2.push_back(s->pose.pose.orientation.z);
            dataOdom_2.push_back(s->pose.pose.orientation.w);
	}
      }
      else if (m.getTopic() == pathTopic) {
	nav_msgs::Path::ConstPtr s = m.instantiate<nav_msgs::Path>();
	if (s != NULL) {
	  //
	  for (const auto & ss : s->poses) {
            dataPath.push_back(ss.header.stamp.toSec());
            dataPath.push_back(ss.pose.position.x);
            dataPath.push_back(ss.pose.position.y);
            dataPath.push_back(ss.pose.position.z);
            dataPath.push_back(ss.pose.orientation.x);
            dataPath.push_back(ss.pose.orientation.y);
            dataPath.push_back(ss.pose.orientation.z);
            dataPath.push_back(ss.pose.orientation.w);
	  }
	}
      }
      else {
	// do nothing
      }

    }

    bag.close();
    // Debug message
    ROS_INFO("Done processing all bag");

    // ====================================================================
    // ==========              IMU DATA                  ==================
    // ==================================================================== 
//     string pathMat;
//     if(!pathParent.empty()) {
//         pathMat = pathParent+"/"+p.stem().string()+".mat";
//     } else {
//         pathMat = p.stem().string()+".mat";
//     }
//     ROS_INFO("MAT Path is: %s", pathMat.c_str());
//     if (!writeToMat(dataOdom_1, pathMat, "arr_est")) {
//       ROS_ERROR("Failed to save arr_est to mat!");
//       return EXIT_FAILURE;
//     }
//     if (!writeToMat(dataOdom_2, pathMat, "arr_act")) {
//       ROS_ERROR("Failed to save arr_act to mat!");
//       return EXIT_FAILURE;
//     }
//     if (!writeToMat(dataPath, pathMat, "arr_plan")) {
//       ROS_ERROR("Failed to save arr_plan to mat!");
//       return EXIT_FAILURE;
//     }
    
    string pathOdom_1, pathOdom_2, pathMat;
    if(!pathParent.empty()) {
         pathOdom_1 = pathParent+"/"+p.stem().string()+"_arr_est.txt";
         pathOdom_2 = pathParent+"/"+p.stem().string()+"_arr_act.txt";
         pathMat = pathParent+"/"+p.stem().string()+"_arr_plan.txt";
     } else {
         pathOdom_1 = p.stem().string()+"_arr_est.txt";
         pathOdom_2 = p.stem().string()+"_arr_act.txt";
         pathMat = p.stem().string()+"_arr_plan.txt";
     }
    writeToText(dataOdom_1, pathOdom_1);
    writeToText(dataOdom_2, pathOdom_2);
    writeToText(dataPath, pathMat);
    
    // Debug message
//     ROS_INFO("Done saving all mat");
    ROS_INFO("Done saving all text");
    
    return EXIT_SUCCESS;
}


