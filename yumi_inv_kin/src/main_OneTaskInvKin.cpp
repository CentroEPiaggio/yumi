
#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "OneTaskInvKin.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

  ros::init(argc, argv, "OneTaskInvKin_node");

  OneTaskInvKin Obj;
  // std::cout<<"sono qui"<<std::endl;
  double rate_200Hz = 200.0;
  ros::Rate r_200HZ(rate_200Hz);
  Obj.dt_ = 1.0/rate_200Hz;


  while(ros::ok())
  {
    Obj.run();
    ros::spinOnce();
    r_200HZ.sleep();
        
  }// end while()
return 0;
}