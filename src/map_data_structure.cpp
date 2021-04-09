#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

octomap::OcTree* map_octree;
bool map_updated=false;

void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg) {
  if (msg->data.size() == 0) {
    return;
  }
  delete map_octree;
  map_octree=(octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  map_updated=true;
  return;
}

void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg) {
  if (msg->data.size() == 0) {
    return;
  }
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  map_updated = true;
  return;
}

int main(int argc, char **argv) {
  // Node declaration
  ros::init(argc, argv, "write_octomap_to_file_new");
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_octomap_binary=n.subscribe("octomap_binary", 1, CallbackOctomapBinary);
  ros::Subscriber sub_octomap_full=n.subscribe("octomap_full", 1, CallbackOctomapFull);

  bool message_received=false;
  ros::Rate r(1.0); // 1 Hz
  while(ros::ok() && !(map_updated)) {
    r.sleep();
    ros::spinOnce();
    ROS_INFO("Hello World!");
    if(map_updated) {
	    ROS_INFO("Receved map of size %d",(int)map_octree->size());
      generate_2d();
	    map_updated=false;
    }
  }
  return 0;
}
