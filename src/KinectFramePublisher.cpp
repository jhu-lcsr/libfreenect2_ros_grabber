// ros stuff
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_publisher");
  ros::NodeHandle npc;
  ros::Rate r(100);
  tf::TransformBroadcaster br;
  while (ros::ok())
	{
	  	br.sendTransform(
	  		tf::StampedTransform(
	  			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
	  		ros::Time::now(), "/world", "/kinect_camera"));
	  	r.sleep();
	}
}