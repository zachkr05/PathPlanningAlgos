#include ""
#include <stdio>



//clamped bspline function

// smoothing function

//generate trajectory
void generateTrajectory(){


}

bool generateTrajCb(path_planning::GenerateTraj::Request &req, path_planning::GenerateTraj::Response &res){

  //parse
  
  //send to generateTrajectory

  //smooth

  //resample by arclength

  //send the eigens

}


bool modifyTrajCb(path_planning::GenerateTraj::Request &req, path_planning::GenerateTraj::Response &res){

  //parse

  //smooth

  //resample by arclength

  //send the eigens

}

int main(int argc, char **argv){


  ros::init(argc, argv, "GPF");
  ros::NodeHandle nh;

  //Host the generate traj srv
  ros::ServiceServer generate_traj =  nh.advertiseService("/generate_traj", generateTrajCb);
  ros::serviceServer modify_traj = nh.advertiseService("/modify_traj", modifyTrajCb)

  ros::spin();
  return 0;
}

