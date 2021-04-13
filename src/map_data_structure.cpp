#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "/home/dylan/catkin_ws/src/map_data_structure/src/CImg/CImg.h"
#include <map>
#include <vector>

octomap::OcTree* octree;
bool map_updated=false;

void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg) {
  if (msg->data.size() == 0) {
    return;
  }
  delete octree;
  octree=(octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  map_updated=true;
  return;
}

void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg) {
  if (msg->data.size() == 0) {
    return;
  }
  delete octree;
  octree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  map_updated = true;
  return;
}

//type of elements in hashmap
struct map_value_t {
  double n;
  double var;
  double mean;
};
typedef struct map_value_t map_value;

void viewMap() {

  size_t map_size=octree->size();
  std::cout << "MAP OBTAINED\n";
  double res=octree->getResolution();
  std::cout << "RESOLUTION: " << res << "\n";

  double x_dim_min,y_dim_min,z_dim_min;
  octree->getMetricMin(x_dim_min,y_dim_min,z_dim_min);
  std::cout << "X_DIM_MIN: " << x_dim_min << "\nY_DIM_MIN: " << y_dim_min << "\nZ_DIM_MIN: " << z_dim_min << "\n";

  double x_dim_max,y_dim_max,z_dim_max;
  octree->getMetricMax(x_dim_max,y_dim_max,z_dim_max);

  std::cout << "X_DIM_MAX: " << x_dim_max << "\nY_DIM_MAX: " << y_dim_max << "\nZ_DIM_MAX: " << z_dim_max << "\n";
  
  int x_vox_dim=(x_dim_max-x_dim_min)/res;
  int y_vox_dim=(y_dim_max-y_dim_min)/res;
  int z_vox_dim=(z_dim_max-z_dim_min)/res;

  int imgWidth=x_vox_dim;
  int imgHeight=y_vox_dim;

  cimg_library::CImg<float> img(imgWidth,imgHeight,1,3,0);
  const float color[]={255.0,0.0,0.0};
  octree->expand();
  std::map<std::vector<double>,map_value> posMap;
  octomap::OcTree::leaf_iterator end=octree->end_leafs();
  for(octomap::OcTree::leaf_iterator it=octree->begin_leafs();it!=end;++it) {
    double x=it.getX();
    double y=it.getY();
    double z=it.getZ();
    if(x>=x_dim_min && x<=x_dim_max && y>=y_dim_min && y<=y_dim_max && z>=z_dim_min && z<=z_dim_max) {
      double occ=it->getOccupancy();
      if(occ>0.5) {
        std::vector<double> pos{x,y};
        //check if posMap[pos] is not defined
        if(posMap.find(pos)==posMap.end()) {
          double n,var,mean;
          n=1;  
          var=0;
          mean=z;
          map_value newMapValue=map_value();
          newMapValue.n=n;
          newMapValue.var=var;
          newMapValue.mean=mean;
          posMap[pos]=newMapValue;
        }
        else {
          double n,var,mean;
          double n_old=posMap[pos].n;
          double var_old=posMap[pos].mean;
          double mean_old=posMap[pos].var;
          map_value newMapValue=map_value();
          n=n_old+1;
          mean=((mean_old*(n-1)) + z)/n;
          var=((var_old*(n-1)) + ((z-mean)*(z-mean)) )/n;
          newMapValue.n=n;
          newMapValue.var=var;
          newMapValue.mean=mean;

          posMap[pos]=[n,var,mean];
        }
      }
    }
  }
}




  /*
  int x_voxel_pos=(int)(x-x_dim_min)/res;
  int y_voxel_pos=(int)(x-x_dim_min)/res;
  img.draw_point(x_voxel_pos,y_voxel_pos);
  */
  /*
  std::cout << "X Set Size: " << x_pos_set.size() << "\n";
  std::set<double>::iterator it=x_pos_set.begin();
  std::cout << "X_POS_SET: " << "\n";
  while (it != x_pos_set.end()) {
    std::cout << (*it) << "\n";
    it++;
  }
  */
   //img.save("/home/dylan/catkin_ws/src/map_data_structure/newImg.bmp");



  


int main(int argc, char **argv) {
  // Node declaration
  ros::init(argc, argv, "write_octomap_to_file_new");
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_octomap_binary=n.subscribe("octomap_binary", 1, CallbackOctomapBinary);
  ros::Subscriber sub_octomap_full=n.subscribe("octomap_full", 1, CallbackOctomapFull);

  bool message_received=false;
  ros::Rate r(1.0); // 1 Hz
  while(ros::ok() && !map_updated) {
    r.sleep();
    ros::spinOnce();
    std::cout << "Waiting for map..\n";
    if(map_updated) {
      viewMap();
    }
   
  }
  //after receiving map

  return 0;
}
