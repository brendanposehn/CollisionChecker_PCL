#include "vis_tools.h"

using namespace std::chrono_literals;

void simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string windowName)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (windowName));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
      std::this_thread::sleep_for(100ms);
  }   
  return;
}



void test_fxn(std::string msg)
{
  std::cout << msg << std::endl;
  return;
}