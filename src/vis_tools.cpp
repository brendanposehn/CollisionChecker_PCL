#include "vis_tools.h"

// pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
// {
//   // // --------------------------------------------
//   // // -----Open 3D viewer and add point cloud-----
//   // // --------------------------------------------
//   // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   // viewer->setBackgroundColor (0, 0, 0);
//   // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//   // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//   // viewer->addCoordinateSystem (1.0);
//   // viewer->initCameraParameters ();
//   // return (viewer);
// }
// build causes error

void test_fxn(std::string msg)
{
  std::cout << msg << std::endl;
  return;
}