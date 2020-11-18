#include <iostream>

#include <string>

#include <igl/opengl/glfw/Viewer.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <math.h>

#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
 #include <pcl/common/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Geometry> 

#include "vis_tools.h"

int main(){

    // we are using cpp 11 standards

    pcl::ScopeTime scope_time ("Test loop");

    std::string meshfile ("/home/brend/BCCancer/meshes/boy/fromWindows/doll_merged_mesh.ply");
    pcl::PolygonMesh::Ptr polymesh (new pcl::PolygonMesh);

    std::string cloudfile ("/home/brend/BCCancer/meshes/jermey/j_cloud.pcd");
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); //this is fine
    pcl::io::loadPCDFile (cloudfile, *cloud);
    
    // simpleVis(cloud);

    Eigen::MatrixXf body_points(4, 6); //deff size

    //for jeremy
    Eigen::Vector4f left_shoulder(-.101731, -.0520700, -.650879, 1);
    Eigen::Vector4f right_shoulder(-.287711, .169372, -.801845, 1);
    Eigen::Vector4f upper_spine(.209333, .252782, -.918178, 1);
    Eigen::Vector4f lower_spine(.304443, .270415, -.988594, 1);
    Eigen::Vector4f center_chest(-.217534, 0.053130, -.708382, 1);
    Eigen::Vector4f center_shoulder_side(-.079892, -.160967, -.652748, 1);

    //TODO
    //add method for picking these in this application
    //make external file for rotations

    enum points {LEFTSHOULDER, RIGHTSHOULDER, UPPERSPINE, LOWERSPINE, CENTERCHEST, CENTERSHOULDERSIDE};

    body_points.col(LEFTSHOULDER) = left_shoulder;
    body_points.col(RIGHTSHOULDER) = right_shoulder;
    body_points.col(UPPERSPINE) = upper_spine;
    body_points.col(LOWERSPINE) = lower_spine;
    body_points.col(CENTERCHEST) = center_chest;
    body_points.col(CENTERSHOULDERSIDE) = center_shoulder_side; 

    //we want to have the centre chest at the origin
    //but we want to shift it in the z a bit so the origin is at the z-level of the mid shoulder (viewed from the side)

    Eigen::Matrix4f trans_mat;
    trans_mat << 1, 0, 0, -body_points(0, CENTERCHEST),
                 0, 1, 0, -body_points(1, CENTERCHEST),
                 0, 0, 1, -body_points(2, CENTERCHEST),
                 0, 0, 0, 1; //fine to define it like this but not when const
        
    body_points = trans_mat*body_points;

    //Still need to test all below

    const pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // Eigen::Affine3f m;
    // m = Eigen::Translation3f(-center_chest(0), -center_chest(1), -center_chest(2)); //dont think we need to do this

    //quite sure that the point cloud types are fine
    pcl::transformPointCloud(*cloud, *translated_cloud, trans_mat);

    simpleVis(translated_cloud, "Translated");

    double theta_x = std::atan((body_points(LEFTSHOULDER, 2) - body_points(RIGHTSHOULDER, 2))/(body_points(LEFTSHOULDER, 1) - body_points(RIGHTSHOULDER, 1)));
    double theta_y = std::atan((body_points(LEFTSHOULDER, 0) - body_points(RIGHTSHOULDER, 0))/(body_points(LEFTSHOULDER, 2) - body_points(RIGHTSHOULDER, 2)));
    double theta_z = std::atan((body_points(LEFTSHOULDER, 1) - body_points(RIGHTSHOULDER, 1))/(body_points(LEFTSHOULDER, 0) - body_points(RIGHTSHOULDER, 0)));

    // now we can produce the rotation matrices

    Eigen::Matrix3f rot_z;
    rot_z << std::cos(theta_z), -std::sin(theta_z), 0,
             std::sin(theta_z), std::cos(theta_z), 0,
             0, 0, 1;
    Eigen::Matrix3f rot_y;
    rot_y << std::cos(theta_y), 0, std::sin(theta_y),
             0, 1, 0,
            -std::sin(theta_y), 0, -std::cos(theta_y);
    Eigen::Matrix3f rot_x;
    rot_x << 1, 0, 0,
             0, std::cos(theta_x), -std::sin(theta_x),
             0, std::sin(theta_x), std::cos(theta_x);
    Eigen::Matrix3f rot_mat_3;
    Eigen::Matrix4f rot_mat_4;
    rot_mat_4 = Eigen::MatrixXf::Zero(4, 4);

    rot_mat_3 = rot_z*rot_y*rot_x;  

    Eigen::Matrix3f rot_mat;
    rot_mat = Eigen::AngleAxisf(-theta_x, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(-theta_y, Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ());
    
    rot_x = Eigen::AngleAxisf(-theta_x, Eigen::Vector3f::UnitX());
    rot_y = Eigen::AngleAxisf(-theta_y, Eigen::Vector3f::UnitY());
    rot_z = Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f rot_x_;
    rot_x_ = Eigen::MatrixXf::Zero(4, 4);
    rot_x_.topLeftCorner(3,3) = rot_x;
    rot_x_(3, 3) = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud_x (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*translated_cloud, *rotated_cloud_x, rot_x_);
    simpleVis(rotated_cloud_x, "Rotated X Only"); 

    Eigen::Matrix4f rot_y_;
    rot_y_ = Eigen::MatrixXf::Zero(4, 4);
    rot_y_.topLeftCorner(3,3) = rot_y;
    rot_y_(3, 3) = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud_y (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*translated_cloud, *rotated_cloud_y, rot_y_);
    simpleVis(rotated_cloud_y, "Rotated Y Only"); 

    Eigen::Matrix4f rot_z_;
    rot_z_ = Eigen::MatrixXf::Zero(4, 4);
    rot_z_.topLeftCorner(3,3) = rot_z;
    rot_z_(3, 3) = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud_z (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*translated_cloud, *rotated_cloud_z, rot_z_);
    simpleVis(rotated_cloud_z, "Rotated Z Only"); 

    rot_mat_4.topLeftCorner(3,3) = rot_mat;
    rot_mat_4(3, 3) = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*translated_cloud, *rotated_cloud, rot_mat_4);

    simpleVis(rotated_cloud, "All Rotations"); 
    
    // top of chest is at z=0, we want middle chest at z=0
    
    // Eigen::MatrixXf trans_mat_2(4, 4);
    // trans_mat_2 << 1, 0, 0, 0,
    //                0, 1, 0, 0,
    //                0, 0, 1, -body_points(2, CENTERSHOULDERSIDE),
    //                0, 0, 0, 1;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::transformPointCloud (*rotated_cloud, *transformed_cloud, rot_mat);
    // body_points = trans_mat_2*body_points; //not sure if even need this anymore

    // //x is height of person
    // //y is width of person
    // //z is spine to belly button

    // //we can segment the arms from here just by defining planes vertical in z at specific y values

    // size_t num_points = transformed_cloud->size();  

    // pcl::ModelCoefficients::Ptr coefficients_rhs (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers_rhs (new pcl::PointIndices); 
    // coefficients_rhs->values = [0, 1, 0, -body_points(2, -RIGHTSHOULDER)]; //defines plane at y = RIGHTSHOULDER_Y
    // //crete segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.01);

    // seg.setInputCloud (transformed_cloud);
    // seg.segment (*inliers_rhs, *coefficients_rhs);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr body_left_arm (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::copyPointCloud(*transformed_cloud, inliers_rhs, body_left_arm);

    // std::vector<int> inliers_rhs_vec = inliers_rhs->indices;
    // std::vector<int> outliers_rhs_vec(num_points - inliers_rhs_vec.size());
    // //now we can determine what is in and what is not
    // for(int i = 0; i<num_points; i++){
    //     if (std::count(inliers_rhs_vec.begin(), inliers_rhs_vec.end(), i) == 0){
    //         outliers_rhs_vec.push_back(i);
    //     }
    // }
    // //now we need to an outliers PointIndices object
    // pcl::PointIndices::Ptr outliers_rhs (outliers_rhs_vec); //dont think this is right :)
    // //copyPointCloud requires a PointIndices object

    return 0;
}
