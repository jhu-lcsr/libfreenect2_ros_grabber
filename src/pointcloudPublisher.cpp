/*
Copyright 2015, Giacomo Dabisias"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
t (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@Author 
Giacomo. Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
//template<typename Cloud>PCLPointCloud2

int main(int argc, char** argv)
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] [output.ply]\n -processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
  processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;
  if(argc>1){
      int fnpInt;
      ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
      parse_argument (argc, argv, "-processor", fnpInt);
      freenectprocessor = static_cast<processor>(fnpInt);
      
  }
    
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  K2G k2g(freenectprocessor);
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getCloud();

  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); 
  
  //  Initialize ros node stuff
  std::string POINTS_OUT("points_out");
  ros::init(argc, argv, "point_cloud_publisher");
  // std::string POINTS_IN("/camera/depth_registered/points");
  std::string frame_id("/kinect_camera"); //add actual frame 
  ros::NodeHandle npc;
  ros::Publisher pc_pub = npc.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);
  //frame stuff
  //cameraFrame = tf::TransformBroadcaster tBroadcast;
  //tf::Transform transform;
  //transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

  while ((!viewer->wasStopped()) && (ros::ok())) {
    viewer->spinOnce ();
    //using namespace std::chrono;
    //static high_resolution_clock::time_point last;

    //auto tnow = high_resolution_clock::now();
    
   
    cloud = k2g.updateCloud(cloud);
    //auto tpost = high_resolution_clock::now();
    // std::cout << "delta " << duration_cast<duration<double>>(tpost-tnow).count()*1000 << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 
    
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*cloud,output_msg);
    output_msg.header.frame_id = frame_id;
    pc_pub.publish(output_msg);
    ros::spinOnce();
    /*
    if(ply_file_indices.size() > 0 ){
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud,cloud2);
        saveCloud(std::string(argv[ply_file_indices[0]]),cloud2,false,false);
        done = true;
    } */
  }

  k2g.shutDown();
  return 0;
}
