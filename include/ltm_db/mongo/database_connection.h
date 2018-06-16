/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Db-level operations.  Most operations are in message_collection.h
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_MONGO_DATABASE_CONNECTION_H
#define LTM_DB_MONGO_DATABASE_CONNECTION_H

#include <ltm_db/mongo/message_collection.h>
#include <ltm_db/interface/database_connection.h>
#include <boost/shared_ptr.hpp>

namespace ltm_db_mongo
{

class MongoDatabaseConnection : public ltm_db::DatabaseConnection
{
public:
  MongoDatabaseConnection();

  bool setParams(const std::string& host, unsigned port, float timeout);

  bool setTimeout(float timeout);

  bool connect();

  bool isConnected();

  void dropDatabase(const std::string& db_name);

  std::string messageType(const std::string& db_name, const std::string& collection_name);

protected:
  boost::shared_ptr<mongo::DBClientConnection> conn_;

  std::string host_;
  unsigned port_;
  float timeout_;

  MessageCollectionHelper::Ptr openCollectionHelper(const std::string& db_name,
                                                    const std::string& collection_name);
};

} // namespace

#endif // include guard
