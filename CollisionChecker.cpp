#include <iostream>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <igl/opengl/glfw/Viewer.h>
#include <string>

int main(){

    // we are using cpp 11 standards

    pcl::ScopeTime scope_time ("Test loop");

    std::string meshfile ("/home/brend/BCCancer/meshes/boy/fromWindows/merged_mesh.ply");
    pcl::PolygonMesh::Ptr polymesh (new pcl::PolygonMesh);
    //what does the '... polymesh = (new pcl::PolygonMesh)' mean ?
    //is it the same as '... polymesh = new pcl::PolygonMesh'?
    pcl::PLYReader Reader;
    Reader.read(meshfile, *polymesh);

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));   
    //getting error here: 'undefined reference to pcl::visualization::PCLVisualizer....
    //might be due to building with wrong cpp source (apparently needs to be c++ 14 standards)

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //this fails too for same reason^^

    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPolygonMesh(*polymesh,"meshes",0);
    // viewer->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();
    // while (!viewer->wasStopped ()){
    //     viewer->spinOnce (100);
    //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // }
    //executes fine now 
}