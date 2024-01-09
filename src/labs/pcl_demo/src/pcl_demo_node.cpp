#include <pcl_demo/pcl_demo.h>


int main(int argc, char **argv){
  ros::init(argc,argv, "pcl_demo_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber pc_sub = nh.subscribe("/r200/camera/depth_registered/points", 1, cloud_callback);

  
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}