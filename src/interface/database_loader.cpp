/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fetch Robotics
 *  All rights reserved.
 *********************************************************************/

/* Author: Connor Brew */

#include <ltm_db/interface/database_loader.h>

namespace ltm_db
{

using std::string;

DatabaseLoader::DatabaseLoader() :
    nh_("~")
{
  initialize();
}

DatabaseLoader::~DatabaseLoader()
{
}

void DatabaseLoader::initialize()
{
  // Create the plugin loader.
  try
  {
    db_plugin_loader_.reset(
        new pluginlib::ClassLoader<DatabaseConnection>("ltm_db", "ltm_db::DatabaseConnection"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating database_connection plugin loader " << ex.what());
  }
}

typename DatabaseConnection::Ptr DatabaseLoader::loadDatabase()
{
  if (!db_plugin_loader_)
  {
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  // Search for the warehouse_plugin parameter in the local namespace of the node, and up the tree of namespaces.
  // If the desired param is not found, make a final attempt to look for the param in the default namespace
  string paramName;
  if (!nh_.searchParam("warehouse_plugin", paramName))
    paramName = "warehouse_plugin";
  string db_plugin;
  if (!nh_.getParamCached(paramName, db_plugin))
  {
    ROS_ERROR("Could not find parameter for database plugin name");
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  DatabaseConnection::Ptr db;
  try
  {
    db.reset(db_plugin_loader_->createUnmanagedInstance(db_plugin));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("Exception while loading database plugin '" << db_plugin << "': " << ex.what() << std::endl);
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  bool hostFound = false;
  bool portFound = false;

  if (!nh_.searchParam("warehouse_host", paramName))
    paramName = "warehouse_host";
  std::string host;
  if (nh_.getParamCached(paramName, host))
  {
    hostFound = true;
  }

  if (!nh_.searchParam("warehouse_port", paramName))
    paramName = "warehouse_port";
  int port;
  if (nh_.getParamCached(paramName, port))
  {
    portFound = true;
  }

  if (hostFound && portFound)
  {
    db->setParams(host, port);
  }

  return db;
}

}
