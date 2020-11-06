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

    //we need some way to align the scans so we now can rotate etc in the right direction
    //maybe take some input from the visualizer defining a line and center of chest

    //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
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

    Eigen::MatrixXf body_points(4, 6); //deff size

    Eigen::Vector4f left_shoulder(-.101731, -.0520700, -.650879, 1);
    Eigen::Vector4f right_shoulder(-.287711, .169372, -.801845, 1);
    Eigen::Vector4f upper_spine(.209333, .252782, -.918178, 1);
    Eigen::Vector4f lower_spine(.304443, .270415, -.988594, 1);
    Eigen::Vector4f center_chest(-.217534, 0.053130, -.708382, 1);
    Eigen::Vector4f center_shoulder_side(-.079892, -.160967, -.652748, 1);

    body_points.col(0) = left_shoulder;
    body_points.col(1) = right_shoulder;
    body_points.col(2) = upper_spine;
    body_points.col(3) = lower_spine;
    body_points.col(4) = center_chest;
    body_points.col(5) = center_shoulder_side; 

    //we want to have the centre chest at the origin
    //but we want to shift it in the z a bit so the origin is at the z-level of the mid shoulder (viewed from the side)

    Eigen::MatrixXf trans_mat(4, 4);
    trans_mat << 1, 0, 0, -center_chest(0),
                 0, 1, 0, -center_chest(1),
                 0, 0, 1, -center_chest(2),
                 0, 0, 0, 1;
        
    for(int i=0; i<=3; i++){
        for(int j=0; j<=5; j++){
            std::cout << " " << body_points(i, j) << " ";
        }
       std::cout << std::endl;
    }

    for(int i=0; i<=5; i++){
        body_points.col(i) = trans_mat*body_points.col(i);
    }

    for(int i=0; i<=3; i++){
        for(int j=0; j<=5; j++){
            std::cout << " " << body_points(i, j) << " ";
        }
       std::cout << std::endl;
    }

    //output is as expected

    double theta_x = std::tan((left_shoulder(2) - right_shoulder(2))/(left_shoulder(1) - right_shoulder(1)));
    double theta_y = std::tan((left_shoulder(0) - right_shoulder(0))/(left_shoulder(2) - right_shoulder(2)));
    double theta_z = std::tan((left_shoulder(1) - right_shoulder(1))/(left_shoulder(0) - right_shoulder(0)));

    // now we can produce the rotation matrices

    Eigen::MatrixXf rot_z(3, 3);
    rot_z << std::cos(theta_z), -std::sin(theta_z), 0,
             std::sin(theta_z), std::cos(theta_z), 0,
             0, 0, 1;
    Eigen::MatrixXf rot_y(3, 3);
    rot_y << std::cos(theta_y), 0, std::sin(theta_y),
             0, 1, 0,
             -std::sin(theta_y), 0, -std::cos(theta_y);
    Eigen::MatrixXf rot_x(3, 3);
    rot_x << 1, 0, 0,
             0, std::cos(theta_x), -std::sin(theta_x),
             0, std::sin(theta_x), std::cos(theta_x);
    Eigen::MatrixXf rot_mat(3, 3);
    rot_mat = rot_z*rot_y*rot_x;    
}