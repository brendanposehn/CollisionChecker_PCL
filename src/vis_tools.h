#ifndef VIS_TOOLS_H
#define VIS_TOOLS_H

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

void simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string windowName);
void test_fxn(std::string msg);

#endif