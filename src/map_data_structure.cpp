#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <list>
#include <math.h> 
#include <visualization_msgs/Marker.h>

// type of each interval in hashmap
struct interval_t {
  bool intervalType; // true if horizontal, false if vertical
  double n;
  double std;
  double mean;
  double occupancy;
  double z_max;
  double z_min; // for testing
  double d; // z_max - z_min
};
typedef struct interval_t interval;

// type of each cell in hashmap
struct map_value_t {
  std::list<interval> intervals;
  std::list<double> *z_vals; // for testing
};
typedef struct map_value_t map_value;

class levelMap {
  public:
  octomap::OcTree* octree;
  std::map<std::vector<double>,map_value> posMap;
  double occThreshold=0.25;
  double res;
  double x_dim_min,y_dim_min,z_dim_min;
  double x_dim_max,y_dim_max,z_dim_max;
  double gap_distance=1; // free space value (m) 
  double thickness_value=0.1; // maximum thickness for horizontal interval (m)

  // constructor
  levelMap() {
  }

  // adds 
  void setOctomap(octomap::OcTree* Octree) {
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

  // checks if pos(x,y,z) is within the map
  bool inMap(double x, double y, double z) {
    if(x>=x_dim_min && x<=x_dim_max && y>=y_dim_min && y<=y_dim_max && z>=z_dim_min && z<=z_dim_max) {
      return true;
    }
    return false;
  }

  // checks if voxel is occupied
  bool isOcc(double occ) {
    if(occ>occThreshold) {
      return true;
    }
    return false;
  }


  // Inefficient helper function O(n) for generating map from voxel pos
  // Does not require z values at (x,y) to inserted in increasing order 
  // inserts a vector (x,y) as a key with corresponding map_value into hashmap 
  void insertVoxelAny(double x, double y, double z) {
  }

  
  // Efficient helper function for generating map from octomap voxel position
  // Requires z values to be inserted in increasing order at (x,y)
  // inserts a vector (x,y) as a key with corresponding map_value into hashmap 
  void insertVoxelIn(double x, double y, double z) {
    std::vector<double> pos{x,y}; // only (x,y) used for hash
    
    //new values to be assigned
    double n,std,mean,d;
    map_value newMapValue;

    // if (x,y) is NOT in the map
    if(posMap.find(pos)==posMap.end()) {
      newMapValue=map_value();
      // generate the first interval
      interval newInterval;
      newMapValue.intervals.push_back(newInterval);

      // assign values to new interval
      newMapValue.intervals.back().z_max=z;
      newMapValue.intervals.back().z_min=z;

      n=1; std=0; mean=z,d=0;
      newMapValue.intervals.back().n=1;
      newMapValue.intervals.back().std=std;
      newMapValue.intervals.back().mean=z;
      newMapValue.intervals.back().d=d;

      // New 1 voxel interval is horizontal (true)
      newMapValue.intervals.back().intervalType=true;

      //testing
      newMapValue.z_vals=new std::list<double>; // makes list for z_values
      newMapValue.z_vals->push_back(z); 
    }

    // if (x,y) is in the map
    else {
      newMapValue=posMap[pos];
      double z_max=newMapValue.intervals.back().z_max;
      
      //check if greater than last z value
      assert(z>=z_max); 

      // add a new interval if there is a gap
      if((z-z_max)>=gap_distance) {
        
        interval newInterval;


        newMapValue.intervals.push_back(newInterval);
        
        // add z to new interval
        newMapValue.intervals.back().z_min=z;
        newMapValue.intervals.back().z_max=z;

        n=1; std=0; mean=z,d=0;
        newMapValue.intervals.back().n=n;
        newMapValue.intervals.back().std=std;
        newMapValue.intervals.back().mean=mean;
        newMapValue.intervals.back().d=0;
        // New 1 voxel interval is horizontal (true)
        newMapValue.intervals.back().intervalType=true;

      }
      // no gap so update existing interval
      else {
        // if interval height > thickness_value 
        // change interval type to vertical (false) 
        // and update mean to z_max and variance to var of
        if(z - newMapValue.intervals.back().z_min > thickness_value) {
          // update existing interval
          newMapValue.intervals.back().z_max=z;
          newMapValue.intervals.back().intervalType=false; 

          n=newMapValue.intervals.back().n + 1;
          std=0;
          mean=z;
          d=z - newMapValue.intervals.back().z_min;

          newMapValue.intervals.back().n = n;
          newMapValue.intervals.back().std=0.5; // ? (set to dist from mean)
          newMapValue.intervals.back().mean=z; // set mean=z_max
          newMapValue.intervals.back().d = d;
        }

        else {
          // retrieve old values
          double n_old=newMapValue.intervals.back().n, std_old=newMapValue.intervals.back().std, mean_old=newMapValue.intervals.back().mean;

          // update existing interval
          newMapValue.intervals.back().z_max=z;

          n=n_old+1;
          mean=((mean_old*(n-1)) + z)/n;
          double var_old=std_old * std_old; //var=std^2
          double var=((var_old*(n-1)) + ((z-mean)*(z-mean)))/n;
          std=sqrt(var);

          newMapValue.intervals.back().n=n;
          newMapValue.intervals.back().std=std;
          newMapValue.intervals.back().mean=mean;
        }
      }
      // testing 
      newMapValue.z_vals->push_back(z); 
    }
    posMap[pos]=newMapValue;
  }

  // generates map from an octomap so setOctomap() must be called first
  // creates a cell in the hash map for each unique (x,y)
  void genMap() {
    octree->expand();
    octomap::OcTree::leaf_iterator end=octree->end_leafs();
    for(octomap::OcTree::leaf_iterator it=octree->begin_leafs();it!=end;++it) {
      double x=it.getX(), y=it.getY(), z=it.getZ();
      if(x>=x_dim_min && x<=x_dim_max && y>=y_dim_min && y<=y_dim_max && z>=z_dim_min && z<=z_dim_max) {
        double occ=it->getOccupancy();
        if(isOcc(occ) && inMap(x,y,z)) {
          insertVoxelIn(x,y,z);
        }
      }
    }
    std::cout << "Map Generated\n";
    testing();
  }
  

  void printList(std::list<double> doubleList) {
    std::cout << "[";
    int i=0;
    int listLen=doubleList.size();
    for (double item : doubleList) {
      std::cout << item;
      if(listLen!=1 && i!=(listLen-1)) {
        std::cout << ",";
      }
      i++;
    }
    std::cout << "]\n";
  }

  // takes in array of 8 empty vectors
  // fills array with vector of neighbor positions
  // array not declared locally and returned since stack pointer is undefined and heap is slow 
  // EX: 
  // std::vector<double> validNeighborPos[8]
  // getNeighbors(1,1,1,validNeighborPos);
  void getNeighbors(double x, double y, double z,std::vector<double> validNeighborPos[8]) {
    std::cout << "Searching for Neigbors of: (" << x << "," << y << "," << z << ") \n";
    double x_deltas_coef[8]={1,1,0,-1,-1,-1,0,1};
    double y_delta_coef[8]={0,1,1,1,0,-1,-1,-1};
    double x_delta, y_delta;
    for(int i=0;i<8;i++) {
      x_delta=x_deltas_coef[i] * res;
      y_delta=y_delta_coef[i] * res;
      map_value* neighborCell=getMapValue(x + x_delta,y + y_delta,z);
      if(neighborCell!=NULL) {
        // iterate through intervals and check if z is within 2 std of a mean
        std::list<interval>* neighborIntervals=&(neighborCell->intervals);
        std::list<interval>::iterator interval_it;
        for(interval_it = neighborIntervals->begin(); interval_it != neighborIntervals->end(); ++interval_it) {
          std::cout << "(" << (x+x_delta) << "," << (y+y_delta) << ")\n"; 
          double interval_mean = interval_it->mean;
          double interval_std = interval_it->std;
          std::cout << "Mean: " << interval_mean << "\n";
          std::cout << "Std: " << interval_std << "\n";
          // check if z is within 2 std of mean
          if(z >= interval_mean-(2*interval_std) && z <= interval_mean+(2*interval_std)) {
            validNeighborPos[i]={x + x_delta, y + y_delta, z}; // unsure about z to return for neighbor?
          }
        }
      }
    }
  }

  // returns pointer to map_value
  // NULL indicates the value was not found
  // Does NOT need to be freed after
  map_value* getMapValue(double x, double y, double z) {
    std::vector<double> searchPos{x,y};
    if(posMap.find(searchPos)!=posMap.end()) {
      return &posMap[searchPos];
    }
    std::cout << "Value not in map\n";
    return NULL;
  }

  // test function to print intervals in cell (x,y)
  void printCell(double x, double y) {
    std::vector<double> pos{x,y};
    map_value cell=posMap[pos];
    std::cout << "---------\n";
    std::cout << "Cell Pos: (" << x << "," << y << ")\n";
    // prints all the z values in cell (x,y)
    std::cout << "Z Vals: ";
    printList(*cell.z_vals);
    int i=1;
    for (interval item : cell.intervals) {
      std::cout << "Interval #" << i << "\n";
      std::cout << "Type: ";
      if(item.intervalType==true) {
        std::cout << "horizontal \n";
      }
      else {
        std::cout << "vertical \n";
      }
      std::cout << "N: " << item.n << "\n";
      std::cout << "Mean: " << item.mean << "\n";
      std::cout << "STD: " << item.std << "\n";
      std::cout << "Z Min: " << item.z_min << "\n";
      std::cout << "Z Max: " << item.z_max << "\n";
      std::cout << "D: " << item.d << "\n";
      i++;
    }
    std::cout << "---------\n";
  }

  // test function that outputs each cell and its interval(s) with interval information
  void testing() {
    std::cout << "Running Testing \n";
    std::map<std::vector<double>,map_value>::iterator iter;
    // iterate over map cells
    for(iter=posMap.begin(); iter!=posMap.end();++iter) {
      std::vector<double> pos=iter->first;
      double x=pos[0];
      double y=pos[1];
      printCell(x,y);
    }
    // test neighbors
    std::vector<double> validNeighborPos[8];
    getNeighbors(0.825,4.425,2.0,validNeighborPos);
    // print neighbors
    std::cout << "Neighbors: ";
    for(int i=0;i<8;i++) {
      //if vector is not empty
      if(!validNeighborPos[i].empty()) {
        std::cout << "(" << validNeighborPos[i][0] << "," << validNeighborPos[i][1] << ") \n";
      }
    }
  }


  // displays multi-level map in RVIZ
  void displayMap(ros::Publisher marker_pub) {
      visualization_msgs::Marker markerMsg;

      markerMsg.header.frame_id="my_frame";
      markerMsg.header.stamp=ros::Time::now();
      markerMsg.ns="map_data_structure";
      markerMsg.action = visualization_msgs::Marker::ADD;
      markerMsg.pose.orientation.w=1.0;
      markerMsg.type=visualization_msgs::Marker::CUBE_LIST;
    
      markerMsg.scale.x=res;
      markerMsg.scale.y=res;
      markerMsg.scale.z=0.05;
      

      std::cout << "Displaying Intervals Testing \n";
      std::map<std::vector<double>,map_value>::iterator iter;

      // iterate over map cells and display each interval at (x,y,mean)
      for(iter=posMap.begin(); iter!=posMap.end();++iter) {
        std::vector<double> pos=iter->first;
        geometry_msgs::Point temp;
        temp.x=pos[0];
        temp.y=pos[1];
        std_msgs::ColorRGBA c;
        map_value cell=posMap[pos];
        for (interval item : cell.intervals) {
          temp.z=item.mean;
          markerMsg.points.push_back(temp);
          // color plane red if interval is vertical 
          if(item.intervalType==false) {
            c.r=1; c.g=0; c.b=0; c.a=1;
          }
          // color plane yellow if interval is horizontal
          else {
            c.r=1; c.g=1; c.b=0; c.a=1;
          }
          markerMsg.colors.push_back(c);
        }
        
      }

      //markerMsg.lifetime = ros::Duration();
      while (marker_pub.getNumSubscribers() < 1) {
        if(!ros::ok()) {
          return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }
      marker_pub.publish(markerMsg);
      markerMsg.points.clear();
      markerMsg.colors.clear();      

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
  
  // Publisher
  ros::Publisher marker_pub=n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  bool message_received=false;
  
  levelMap newMap;
  ros::Rate r(1.0); // 1 Hz
  while(ros::ok() && !map_updated) {
    r.sleep();
    ros::spinOnce();
    std::cout << "Waiting for map..\n";
    if(map_updated) {
      // create map
      newMap=levelMap();
      newMap.setOctomap(octree);
      newMap.genMap();
    }
  }
  // display map in rviz
  while(ros::ok()) {
    r.sleep();
    ros::spinOnce();
    std::cout << "Displaying Map...\n";
    newMap.displayMap(marker_pub);
  }
  //after receiving map
  return 0;
}