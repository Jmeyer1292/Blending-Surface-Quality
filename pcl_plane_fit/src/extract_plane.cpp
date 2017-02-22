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

//Defines
#define numPointsMax 300000
#define numRowMax 5000
#define numColMax 5000
#define numPCLPtsMax 30000000
#define numPCLColMax 3
#define numLowPCLThresh -200.0
#define numHighPCLThresh 200.0
#define numMaxIterRansac 1000

class FitPointCloud
{
    public:

    FitPointCloud()
    {
    //Subscribe to "points" pointcloud msg
      sub = nh.subscribe<PointCloud>("points", numPointsMax, &FitPointCloud::callback, this);

    //Publish points_fit and plane_fit
      pcl_pub_fit = nh.advertise<PointCloud> ("points_fit", numPointsMax, &FitPointCloud::callback);
      pcl_pub_Notfit = nh.advertise<PointCloud> ("points_notfit", numPointsMax, &FitPointCloud::callback);
      pcl_linefit = nh.advertise<PointCloud> ("line_fit", numPointsMax, &FitPointCloud::callback);
    }

    void callback(const PointCloud::ConstPtr& msg)
    {
        int nr_points = (int) msg->points.size ();
        std::cout<<nr_points<<" Points"<<std::endl;

        float planeFitThreshold = 0.5;

                PointCloud::Ptr cloud_filtered (new PointCloud),
                                cloud_p (new PointCloud),
                                cloud (new PointCloud),
                                finalline (new PointCloud);

                std::cout<<"Cloud: width = "<<msg->width<<", height = "<<msg->height<<std::endl;


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (numMaxIterRansac);
  seg.setDistanceThreshold (planeFitThreshold);

  while (msg->points.size () > 0.3 * nr_points)
    {

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

  // While 30% of the original cloud is still there

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (msg);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (msg);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
	pcl_pub_fit.publish(cloud_p);


	//Line Fit
	std::vector<int> lineInliers;

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (msg));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (planeFitThreshold);
    ransac.computeModel();
    ransac.getInliers(lineInliers);

    // copies all lineInliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*msg, lineInliers, *finalline);
    pcl_linefit.publish(finalline);
    ros::spin ();
  }
  }

  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pcl_pub_fit;
    ros::Publisher pcl_pub_Notfit;
    ros::Publisher pcl_linefit;
};

int main (int argc, char** argv)
{

  //Initialize the ROS node
  ros::init(argc, argv, "sub_pcl");

  //Create point cloud plane fitting object
  FitPointCloud FPCLObject;

  ros::spin ();

  return (0);
}
