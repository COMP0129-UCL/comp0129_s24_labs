#include <moveit_solution/moveit_solution.h>

int main(int argc, char** argv)
{
  // initialise ros and the node
  ros::init(argc, argv, "moveit_solution_node");
  ros::NodeHandle nh("~");
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create the service object to handle all callbacks
  SrvClass myobject(nh);

  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();

    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

