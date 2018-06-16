/*
 * Copyright (c) 2015, Fetch Robotics
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Implementation of template methods of DatabaseConnection
 * Only to be included by database_connection.h
 *
 * \author Connor Brew
 */

namespace ltm_db
{

template<class M>
  MessageCollection<M> DatabaseConnection::openCollection(const std::string& db_name,
                                                          const std::string& collection_name)
  {
    if (!isConnected())
      throw DbConnectException("Cannot open collection.");
    return MessageCollection<M>(openCollectionHelper(db_name, collection_name));
  }

template<class M>
  typename MessageCollection<M>::Ptr DatabaseConnection::openCollectionPtr(const std::string& db_name,
                                                                           const std::string& collection_name)
  {
    if (!isConnected())
      throw DbConnectException("Cannot open collection.");
    return typename MessageCollection<M>::Ptr(new MessageCollection<M>(openCollectionHelper(db_name, collection_name)));
  }

} // namespace
