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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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

    Eigen::MatrixXf body_points(4, 6); //deff size

    //for jeremy
    Eigen::Vector4f left_shoulder(-.101731, -.0520700, -.650879, 1);
    Eigen::Vector4f right_shoulder(-.287711, .169372, -.801845, 1);
    Eigen::Vector4f upper_spine(.209333, .252782, -.918178, 1);
    Eigen::Vector4f lower_spine(.304443, .270415, -.988594, 1);
    Eigen::Vector4f center_chest(-.217534, 0.053130, -.708382, 1);
    Eigen::Vector4f center_shoulder_side(-.079892, -.160967, -.652748, 1);

    enum points {LEFTSHOULDER, RIGHTSHOULDER, UPPERSPINE, LOWERSPINE, CENTERCHEST, CENTERSHOULDERSIDE}

    body_points.col(LEFTSHOULDER) = left_shoulder;
    body_points.col(RIGHTSHOULDER) = right_shoulder;
    body_points.col(UPPERSPINE) = upper_spine;
    body_points.col(LOWERSPINE) = lower_spine;
    body_points.col(CENTERCHEST) = center_chest;
    body_points.col(CENTERSHOULDERSIDE) = center_shoulder_side; 

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

    // for(int i=0; i<=5; i++){
    //     body_points.col(i) = trans_mat*body_points.col(i);
    // }

    body_points = trans_mat*body_points;
    // instead of above

    for(int i=0; i<=3; i++){
        for(int j=0; j<=5; j++){
            std::cout << " " << body_points(i, j) << " ";
        }
       std::cout << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*source_cloud, *translated_cloud, trans_mat);

    //doesnt matter that we translated above cuz all these are relative
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
    
    //from here we still want to translate upwards
    //we need to be dealing with a point cloud here
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*translated_cloud, *rotated_cloud, rot_mat);
    rot_mat.conservativeResize(4, 4);
    rot_mat(3, 3) = 1;
    body_points = rot_mat*body_points;
    //now that everything is aligned properly we still want to perform one more translation
    //in order to get 
    //currently the top of chest is at z=0, we want middle chest at z=0
    
    Eigen::MatrixXf trans_mat_2(4, 4);
    trans_mat_2 << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, -body_points(2, CENTERSHOULDERSIDE),
                   0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*rotated_cloud, *transformed_cloud, rot_mat);
    body_points = trans_mat_2*body_points; //not sure if even need this anymore

    //x is height of person
    //y is width of person
    //z is spine to belly button

    //we can segment the arms from here just by defining planes vertical in z at specific y values

    size_t num_points = transformed_cloud->size();  

    pcl::ModelCoefficients::Ptr coefficients_rhs (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_rhs (new pcl::PointIndices); 
    coefficients_rhs->values = [0, 1, 0, -body_points(2, -RIGHTSHOULDER)]; //defines plane at y = RIGHTSHOULDER_Y
    //crete segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (transformed_cloud);
    seg.segment (*inliers_rhs, *coefficients_rhs);

    pcl::PointCloud<pcl::PointXYZ>::Ptr body_left_arm (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::copyPointCloud(*transformed_cloud, inliers_rhs, body_left_arm);

    std::vector<int> inliers_rhs_vec = inliers_rhs->indices;
    std::vector<int> outliers_rhs_vec(num_points - inliers_rhs_vec.size());
    //now we can determine what is in and what is not
    for(int i = 0; i<num_points; i++){
        if (std::count(inliers_rhs_vec.begin(), inliers_rhs_vec.end(), i) == 0){
            outliers_rhs_vec.push_back(i);
        }
    }
    //now we need to an outliers PointIndices object
    pcl::PointIndices::Ptr outliers_rhs (outliers_rhs_vec); //dont think this is right :)
    //copyPointCloud requires a PointIndices object






    



}