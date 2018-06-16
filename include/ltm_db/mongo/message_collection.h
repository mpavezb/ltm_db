/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * The MessageCollection class
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_MONGO_MESSAGE_COLLECTION_H
#define LTM_DB_MONGO_MESSAGE_COLLECTION_H

#include <ltm_db/mongo/query_results.h>
#include <ltm_db/interface/message_collection.h>

namespace ltm_db_mongo
{

using ltm_db::Metadata;
using ltm_db::Query;
using ltm_db::MessageCollectionHelper;
using ltm_db::ResultIteratorHelper;

class MongoMessageCollection: public ltm_db::MessageCollectionHelper
{
public:
  MongoMessageCollection(boost::shared_ptr<mongo::DBClientConnection> conn,
                         const std::string& db_name,
                         const std::string& collection_name);

  bool initialize(const std::string& datatype, const std::string& md5);

  /// \post Ensure that there's an index on the given field.
  /// Note that index on _id and creation_time are always created.
  void ensureIndex(const std::string& field);

  /// \brief Insert a ROS message, together with some optional metadata,
  /// into the db
  /// \throws mongo::DBException if unable to insert
  void insert(char* msg, size_t msg_size, Metadata::ConstPtr metadata);

  /// \retval Iterator range over matching messages
  /// \param query A metadata object representing a query.
  /// \param metadata_only If this is true, only retrieve the metadata
  /// (returned message objects will just be default constructed)
  ResultIteratorHelper::Ptr query(Query::ConstPtr query,
                                  const std::string& sort_by,
                                  bool ascending) const;

  /// \brief Remove messages matching query
  unsigned removeMessages(Query::ConstPtr query);
  
  /// \brief Modify metadata
  /// Find message matching \a q and update its metadata using \a m
  /// In other words, overwrite keys in the message using \a m, but
  /// keep keys that don't occur in \a m.
  void modifyMetadata(Query::ConstPtr q, Metadata::ConstPtr m);

  /// \brief Count messages in collection
  unsigned count();

  /// \brief Return name of collection
  std::string collectionName() const;

  Query::Ptr createQuery() const {
    return Query::Ptr(new MongoQuery());
  }

  Metadata::Ptr createMetadata() const {
    return Metadata::Ptr(new MongoMetadata());
  }
  
private:
  void listMetadata(mongo::Query& mquery, std::vector<mongo::BSONObj>& metas);

  inline MongoMetadata& downcastMetadata(Metadata::ConstPtr metadata) const {
    return *(const_cast<MongoMetadata*>(static_cast<const MongoMetadata*>(metadata.get())));
  }

  inline MongoQuery& downcastQuery(Query::ConstPtr query) const {
    return *(const_cast<MongoQuery*>(static_cast<const MongoQuery*>(query.get())));
  }

  boost::shared_ptr<mongo::DBClientConnection> conn_;
  boost::shared_ptr<mongo::GridFS> gfs_;
  const std::string ns_;
  const std::string db_;
  const std::string coll_;
};

} // namespace

#endif // include guard
