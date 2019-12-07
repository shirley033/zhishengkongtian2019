#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_laser");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 1, 0, 0), tf::Vector3(0.01, 0.0, 0.0)),
        //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_link", "velodyne"));     //0.045, 0.0, 0.425
        // ros::Time::now(),"world", "base_link"));
    r.sleep();
  }
}
//w=-0.448074,x=0.893997,y=0,z=-0
