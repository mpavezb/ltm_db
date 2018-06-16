#include <ltm_db/interface/database_loader.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "db_loader_test", ros::init_options::AnonymousName);

  ltm_db::DatabaseLoader dbloader;
  ltm_db::DatabaseConnection::Ptr conn = dbloader.loadDatabase();
  //conn->setParams("localhost", 27017, 10.0);
  conn->setTimeout(10.0);
  if (!conn->connect())
    ROS_ERROR("Failed to connect to DB");

  ros::shutdown();
}
