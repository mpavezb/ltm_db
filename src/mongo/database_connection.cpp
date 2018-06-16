/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Implementation of database_connection.h
 *
 * \author Bhaskara Marthi
 */

#include <ltm_db/mongo/database_connection.h>
#include <pluginlib/class_list_macros.h>

namespace ltm_db_mongo
{

using std::string;

MongoDatabaseConnection::MongoDatabaseConnection() :
  host_("localhost"),
  port_(27017),
  timeout_(60.0)
{
}

bool MongoDatabaseConnection::setParams(const string& host, unsigned port, float timeout)
{
  host_ = host;
  port_ = port;
  timeout_ = timeout;
  return true;
}

bool MongoDatabaseConnection::setTimeout(float timeout)
{
  timeout_ = timeout;
  return true;
}

bool MongoDatabaseConnection::connect()
{
  const string db_address = (boost::format("%1%:%2%") % host_ % port_).str();
  const ros::WallTime end = ros::WallTime::now() + ros::WallDuration(timeout_);

  while (ros::ok() && ros::WallTime::now()<end)
  {
    conn_.reset(new mongo::DBClientConnection());
    try
    {
      ROS_DEBUG_STREAM_NAMED("db_connect", "Attempting to connect to MongoDB at " << db_address);
      conn_->connect(db_address);
      if (!conn_->isFailed())
        break;
    }
    catch (mongo::ConnectException& e)
    {
      ros::Duration(1.0).sleep();
    }
  }
  if (conn_->isFailed())
  {
    ROS_ERROR_STREAM("Unable to connect to the database at '" << db_address << "'. If you just created the database, it could take a while for initial setup.");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED("db_connect", "Successfully connected to the DB");
  return true;
}

bool MongoDatabaseConnection::isConnected()
{
  return ((bool)conn_ && !conn_->isFailed());
}

void MongoDatabaseConnection::dropDatabase(const string& db_name)
{
  if (!isConnected())
    throw ltm_db::DbConnectException("Cannot drop database");
  conn_->dropDatabase(db_name);
}

string MongoDatabaseConnection::messageType(const string& db, const string& coll)
{
  if (!isConnected())
    throw ltm_db::DbConnectException("Cannot look up metatable.");
  const string meta_ns = db+".ros_message_collections";
  std::auto_ptr<mongo::DBClientCursor> cursor = conn_->query(meta_ns, BSON("name" << coll));
  mongo::BSONObj obj = cursor->next();
  return obj.getStringField("type");
}

MessageCollectionHelper::Ptr MongoDatabaseConnection::openCollectionHelper(const std::string& db_name,
                                                                           const std::string& collection_name)
{
  return typename MessageCollectionHelper::Ptr(new MongoMessageCollection(conn_, db_name, collection_name));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( ltm_db_mongo::MongoDatabaseConnection, ltm_db::DatabaseConnection )
