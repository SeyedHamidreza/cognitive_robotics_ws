// 2013-07-29 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
//
// Launches the RACE System
//
#include <boost/thread.hpp>
#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"

// TODO stop only the process requested
bool stopRos()
{
  std::string cmd_rosnode;
  std::string cmd_killall;
  std::string cmd_kill_gaz;
  std::string cmd_kill_gaz_srv;
  bool success = true;

  // ROS is not alive anymore here
  success &= system("killall gzclient &>> /dev/null");

  success &= system("rosnode kill -a &>> /dev/null");
  sleep(5);

  success &= system("killall gzserver &>> /dev/null");
  sleep(5);

  success &= system("killall roscore &>> /dev/null");

  return success;
}

int main(int argc, char **argv)
{
  std::string sesame_repo;
  std::string cmd_scheduler;
  //boost::thread * scheduler_th;
  bool executor = true;

  if (argc >= 2)
    sesame_repo.assign(argv[1]);
  else
  {
    std::cout << std::endl << "Usage: " << argv[0] << " <your locale repository label>" << std::endl;
    std::cout << std::endl << "Running just RACE Gazebo world" << std::endl << std::endl;
    executor = false;
    sleep(3);
  }

  boost::thread roscore_th(system, "roscore");
  sleep(2);
  boost::thread sim_time_th(system, "rosparam set use_sim_time true");
  sleep(1);

  boost::thread gazebo_th(system, "roslaunch race_gazebo_worlds race_world.launch gui:=false furniture:=false");

  sleep(2);
  boost::thread gui_th(system, "rosrun gazebo gui");
  sleep(4);

  boost::thread pr2_th(system, "roslaunch race_gazebo_worlds spawn_pr2.launch ");
  sleep(15);

  // Launch executor only if present
  std::string executor_path = ros::package::getPath("race_plan_executor");
  if (executor_path == "" || executor == false )
  {
    int sys_result;

    std::cout << "Warning: race_plan_executor not found" << std::endl;
    executor = false;
    sys_result = system("roslaunch race_gazebo_worlds spawn_Y2D4.launch");
    sys_result &= system("roslaunch race_gazebo_worlds spawn_mugs.launch");
    sys_result &= system("rosservice call /gazebo/unpause_physics '{}'");
    if (sys_result != 0)
      std::cout << "Warning: unexpected system call result" << std::endl;
  }
  else
  {
    std::cout << "Found race_plan_executor here: " << executor_path << std::endl;
    executor &= true;

    cmd_scheduler.assign("roslaunch race_plan_executor smach_plan_scheduler.launch launch_gazebo:=false sesame_remote:=false gui:=false sesame_repo:='");
    cmd_scheduler.append(sesame_repo);
    cmd_scheduler.append("'");

    //scheduler_th = new boost::thread(system, cmd_scheduler.c_str());
    boost::thread(system, cmd_scheduler.c_str());
  }

  ros::init(argc, argv, "race_run");
  ros::NodeHandle nh;

  while (ros::ok())
  {
    ros::spinOnce();
    sleep(1);
  }

#if 0
  gui_th.join();
  pr2_th.join();

  gazebo_th.join();
  roscore_th.join();

  if (scheduler_th != NULL)
  {
    scheduler_th->join();
    //free(scheduler_th);
  }
#endif

  sleep(10);
  stopRos();

  return 0;
}
