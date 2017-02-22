#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <stdlib.h>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//#Defines
#define numPointsMax 300000
#define numRowMax 5000
#define numColMax 5000
#define numPCLPtsMax 30000000
#define numPCLColMax 3
#define numLowPCLThresh -50.0
#define numHighPCLThresh 200.0
#define numMaxIterRansac 1000

int main (int argc, char** argv)
{
  //Create a point cloud msg
  PointCloud::Ptr msg (new PointCloud);

  const std::string homeDir = getenv("HOME");

  //Initialize the ROS node
  ros::init(argc, argv, "csvtopc2");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<PointCloud> ("points", numPointsMax);

  msg->header.frame_id = "/PlateScan_frame";
  msg->height = 1;
  msg->width = numPointsMax;

  const std::string PlateScanPath = "PlateScans-selected/Scan_16_09_12_short_plate_de_slagged_2.csv";

  float** data = new float*[numRowMax];

  for(int i = 0; i < numRowMax; ++i)
      data[i] = new float[numColMax];

  std::string my_str = homeDir+"/"+PlateScanPath ;

  const char* csvPath = my_str.c_str();

  std::ifstream file(csvPath);

  //Read the csv file and push it into msg pc2
  for(int row = 1; row < numRowMax; ++row)
  {
        if ( !file.good() )
            break;
        std::string line;
        std::getline(file, line);

        std::stringstream iss(line);

        for (int col = 1; col < numColMax; ++col)
        {

            if ( !iss.good() )
                break;

            std::string val;
            std::getline(iss, val, ',');
            std::stringstream convertor(val);

            convertor >> data[row][col];

            if(data[row][col] < numHighPCLThresh &&
               data[row][col] > numLowPCLThresh)
               {
                  msg->points.push_back (pcl::PointXYZ(row,
                                                       col,
                                                       data[row][col]));
               }
        }
  }

  ros::Rate loop_rate(1);
  int nr_points = (int) msg->points.size ();

  while(nh.ok())
  {
    //Publish the point cloud
    pcl_pub.publish(msg);
    loop_rate.sleep ();
  }

  return (0);
}
