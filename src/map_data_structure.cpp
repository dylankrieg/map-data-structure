#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include "/home/dylan/catkin_ws/src/map_data_structure/src/CImg/CImg.h"


// type of elements in hashmap
struct map_value_t {
  double n;
  double var;
  double mean;
  double occupancy;
};
typedef struct map_value_t map_value;

class levelMap {
  public:
  octomap::OcTree* octree;
  std::map<std::vector<double>,map_value> posMap;
  double occThreshold=0.5;
  double res;
  double x_dim_min,y_dim_min,z_dim_min;
  double x_dim_max,y_dim_max,z_dim_max;

  // constructor
  levelMap(octomap::OcTree* Octree) {
      octree=Octree;
      updateMapParams();
  }

  // updates the resolution and dimensions of the tree
  void updateMapParams() {
    res=octree->getResolution();
    std::cout << "Resolution: " << res << "\n";

    octree->getMetricMin(x_dim_min,y_dim_min,z_dim_min);
    std::cout << "X Dim Min: " << x_dim_min << "\nY Dim Min: " << y_dim_min << "\nZ Dim Min: " << z_dim_min << "\n";
    
    octree->getMetricMax(x_dim_max,y_dim_max,z_dim_max);
    std::cout << "X Dim Max: " << x_dim_max << "\nY Dim Max: " << y_dim_max << "\nZ Dim Max: " << z_dim_max << "\n";
  }

  // updates the hash map value in the multi-level map for each unique (x,y,level)
  void genMap() {
    octree->expand();
    octomap::OcTree::leaf_iterator end=octree->end_leafs();
    for(octomap::OcTree::leaf_iterator it=octree->begin_leafs();it!=end;++it) {
      double x=it.getX(), y=it.getY(), z=it.getZ();
      double level=0.0;
      if(x>=x_dim_min && x<=x_dim_max && y>=y_dim_min && y<=y_dim_max && z>=z_dim_min && z<=z_dim_max) {
        double occ=it->getOccupancy();
        if(occ>occThreshold) {
          std::vector<double> pos{x,y,level};
          map_value newMapValue=map_value();
          double n,var,mean;
          // check if (x,y,level) is in the map
          if(posMap.find(pos)==posMap.end()) {
            n=1; var=0; mean=z;
            newMapValue.n=n; newMapValue.var=var; newMapValue.mean=mean;
            newMapValue.occupancy=occ;
          }
          else {
            double n_old=posMap[pos].n, var_old=posMap[pos].mean, mean_old=posMap[pos].var;
            n=n_old+1;
            mean=((mean_old*(n-1)) + z)/n;
            var=((var_old*(n-1)) + ((z-mean)*(z-mean)))/n;
            newMapValue.n=n; newMapValue.var=var; newMapValue.mean=mean;
            newMapValue.occupancy=occ;
          }
          // add (x,y,level) data to map
          posMap[pos]=newMapValue;
        }
      }
    }
    std::cout << "Map Generated\n";
  }

  // default level is 0.0
  // z is not used currently as level is fixed
  // returns pointer to map_value (this is only so NULL can be used for not found)
  // Does NOT need to be freed after
  map_value* getMapValue(double x, double y, double z) {
    double level=0.0;
    std::vector<double> searchPos{x,y,0.0};
    if(posMap.find(searchPos)!=posMap.end()) {
      return &posMap[searchPos];
    }
    std::cout << "Value not in map\n";
    return NULL;
  }
};

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
      // create map
      levelMap newMap=levelMap(octree);

      // generate map
      newMap.genMap(); 
    }
  }
  //after receiving map
  return 0;
}
