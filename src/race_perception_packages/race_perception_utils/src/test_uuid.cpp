
#include <ros/ros.h>
#include <bondcpp/bond.h>
#include <uuid/uuid.h>

static std::string makeUUID()
{
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}


int main(int argc, char** argv)
{
    //init the ROS node
    ros::init(argc, argv, "test_uuid");

    //std::string id = generateUniqueId();
    //std::string id = "ola1";
    std::string id = makeUUID();
    // Sends id to B using a service or action
    bond::Bond bond("example_bond_topic", id);
    bond.start();
    if (!bond.waitUntilFormed(ros::Duration(1.0)))
    {
        ROS_ERROR("ERROR!");
        return false;
    }

    // ... do things with B ...
    bond.waitUntilBroken(ros::Duration(1.0));
    printf("B has broken the bond\n");
    //RobotHead head;
    ////head.shakeHead(3);
    //head.pointAtTable(4);
}
