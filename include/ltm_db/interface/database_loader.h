/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fetch Robotics
 *  All rights reserved.
 *********************************************************************/

/* Author: Connor Brew */

#ifndef LTM_DB_INTERFACE_DATABASE_LOADER_
#define LTM_DB_INTERFACE_LOADER_

#include <boost/scoped_ptr.hpp>
#include <ltm_db/interface/database_connection.h>
#include <pluginlib/class_loader.h>

namespace ltm_db
{

class DBConnectionStub : public DatabaseConnection
{
public:
  bool setParams(const std::string& host, unsigned port, float timeout)
  {
    return false;
  }
  bool setTimeout(float timeout)
  {
    return false;
  }
  bool connect()
  {
    return false;
  }
  bool isConnected()
  {
    return false;
  }
  void dropDatabase(const std::string& db_name)
  {
    throw ltm_db::DbConnectException("Database is stub");
  }
  std::string messageType(const std::string& db_name, const std::string& collection_name)
  {
    throw ltm_db::DbConnectException("Database is stub");
  }
protected:
  typename MessageCollectionHelper::Ptr openCollectionHelper(const std::string& db_name,
                                                             const std::string& collection_name)
  {
  }
  ;
};

/** \brief This class provides the mechanism to connect to a database and reads needed ROS parameters when appropriate. */
class DatabaseLoader
{
public:
  /// \brief Takes a ltm_db DatabaseConnection.  The DatabaseConnection is expected to have already been initialized.
  DatabaseLoader();

  ~DatabaseLoader();

  /// \brief Initialize the DatabaseLoader
  void initialize();

  /** \brief Load a database connection using pluginlib
   Looks for ROS params specifying which plugin/host/port to use. NodeHandle::searchParam()
   is used starting from ~ to look for warehouse_plugin, warehouse_host and warehouse_port. */
  typename DatabaseConnection::Ptr loadDatabase();

private:
  ros::NodeHandle nh_;
  boost::scoped_ptr<pluginlib::ClassLoader<ltm_db::DatabaseConnection> > db_plugin_loader_;
};

}

#endif
