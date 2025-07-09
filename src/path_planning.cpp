#include <stdio>



//move generateTraj to be here then send to the GPF
bool generateTrajCb(){


}

bool modifyTrajCb(){


  //do the modify traj stuff i.e clamped bspline etc


  //send to GPR + MPCC

}

int main(int argc, char **argv){


  ros::init(argc, argv, "path_planning_node");
  ros::NodeHandle nh;

  ros::ServiceServer server = nh.advertiseService("/modify_traj", modifyTrajCb);

  ros::spin();
  return 0;

}
