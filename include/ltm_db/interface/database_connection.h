/*
 * Copyright (c) 2015, Fetch Robotics
 * All rights reserved.
 */

/**
 * \file 
 * 
 * The DatabaseConnection class
 *
 * \author Connor Brew
 */

#ifndef LTM_DB_INTERFACE_DATABASE_CONNECTION_H
#define LTM_DB_INTERFACE_DATABASE_CONNECTION_H

#include <ltm_db/interface/message_collection.h>

namespace ltm_db
{

class DatabaseConnection
{
public:
  /// \brief Set database connection params.
  virtual bool setParams(const std::string& host, unsigned port, float timeout = 60.0) = 0;

  /// \brief Set database connection params.
  virtual bool setTimeout(float timeout) = 0;

  /// Setup the database connection. This call assumes setParams() has been previously called.
  /// Returns true if the connection was succesfully established.
  virtual bool connect() = 0;

  /// Returns whether the database is connected.
  virtual bool isConnected() = 0;

  /// \brief Drop a db and all its collections.
  /// A DbClientConnection exception will be thrown if the database is not connected.
  virtual void dropDatabase(const std::string& db_name) = 0;

  /// \brief Return the ROS Message type of a given collection
  virtual std::string messageType(const std::string& db_name, const std::string& collection_name) = 0;

  /// \brief Open a collection on the DB.  The collection is created if it doesn't exist.
  /// A DbClientConnection exception will be thrown if the database is not connected.
  template<class M>
    MessageCollection<M> openCollection(const std::string& db_name, const std::string& collection_name);

  /// \brief Open a collection on the DB.  The collection is created if it doesn't exist.
  /// A DbClientConnection exception will be thrown if the database is not connected.
  template<class M>
    typename MessageCollection<M>::Ptr openCollectionPtr(const std::string& db_name,
                                                         const std::string& collection_name);

  typedef boost::shared_ptr<DatabaseConnection> Ptr;
protected:
  virtual MessageCollectionHelper::Ptr openCollectionHelper(const std::string& db_name,
                                                            const std::string& collection_name) = 0;
};

} // namespace

#include "impl/database_connection_impl.hpp"

#endif // include guard
