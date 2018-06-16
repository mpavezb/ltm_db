/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Implementation of MongoMessageCollection
 *
 * \author Bhaskara Marthi
 */

#include <ltm_db/mongo/message_collection.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>

namespace ltm_db_mongo
{

using std::string;

MongoMessageCollection::MongoMessageCollection(boost::shared_ptr<mongo::DBClientConnection> conn,
                                               const string& db,
                                               const string& coll) :
  conn_(conn),
  gfs_(new mongo::GridFS(*conn, db)),
  ns_(db+"."+coll),
  db_(db),
  coll_(coll)
{
}


bool MongoMessageCollection::initialize(const std::string& datatype, const std::string& md5)
{
  ensureIndex("creation_time");

  // Add to the metatable
  const string meta_ns = db_+".ros_message_collections";
  if (!conn_->count(meta_ns, BSON("name" << coll_)))
  {
    ROS_DEBUG_NAMED("create_collection", "Inserting info for %s into metatable", coll_.c_str());
    conn_->insert(meta_ns, BSON("name" << coll_ << "type" << datatype << "md5sum" << md5));
  }
  else if (!conn_->count(meta_ns, BSON("name" << coll_ << "md5sum" << md5)))
  {
    ROS_ERROR("The md5 sum for message %s changed to %s. Only reading metadata.", datatype.c_str(), md5.c_str());
    return false;
  }
  return true;
}

void MongoMessageCollection::ensureIndex(const string& field)
{
  conn_->ensureIndex(ns_, BSON(field << 1));
}

void MongoMessageCollection::insert(char* msg, size_t msg_size, Metadata::ConstPtr metadata)
{
  /// Get the BSON and id from the metadata
  mongo::BSONObj bson = downcastMetadata(metadata);
  mongo::OID id;
  bson["_id"].Val(id);

  // Store in message in grid fs
  mongo::BSONObj file_obj = gfs_->storeFile(msg, msg_size, id.toString());

  // Add blob id to metadata and store it in the message collection
  mongo::BSONObjBuilder builder;
  builder.appendElements(bson);
  mongo::OID blob_id;
  file_obj["_id"].Val(blob_id);
  builder.append("blob_id", blob_id);
  mongo::BSONObj entry = builder.obj();
  ROS_DEBUG_NAMED("insert", "Inserting %s into %s", entry.toString().c_str(), ns_.c_str());
  conn_->insert(ns_, entry);
}

ResultIteratorHelper::Ptr MongoMessageCollection::query(Query::ConstPtr query,
                                                        const string& sort_by,
                                                        bool ascending) const
{
  mongo::Query mquery(downcastQuery(query));
  if (sort_by.size() > 0)
    mquery.sort(sort_by, ascending ? 1 : -1);
  ROS_DEBUG_NAMED("query", "Sending query %s to %s", mquery.toString().c_str(), ns_.c_str());
  return typename ResultIteratorHelper::Ptr(new MongoResultIterator(conn_, gfs_, ns_, mquery));
}

void MongoMessageCollection::listMetadata(mongo::Query& mquery, std::vector<mongo::BSONObj>& metas)
{
  MongoResultIterator iter(conn_, gfs_, ns_, mquery);
  while (iter.hasData())
  {
    metas.push_back(iter.metadataRaw());
    iter.next();
  }
}

unsigned MongoMessageCollection::removeMessages(Query::ConstPtr query)
{
  mongo::Query mquery(downcastQuery(query));
  
  std::vector<mongo::BSONObj> metas;
  listMetadata(mquery, metas);

  // Remove messages from db
  conn_->remove(ns_, mquery);

  unsigned num_removed = 0;
  // Also remove the raw messages from gridfs
  for (std::vector<mongo::BSONObj>::iterator it = metas.begin(); it != metas.end(); ++it)
  {
    mongo::OID id;
    (*it)["blob_id"].Val(id);
    gfs_->removeFile(id.toString());
    ++num_removed;
  }
  return num_removed;
}

void MongoMessageCollection::modifyMetadata(Query::ConstPtr q, Metadata::ConstPtr m)
{
  mongo::BSONObj bson = downcastMetadata(m);
  mongo::Query query(downcastQuery(q));

  std::vector<mongo::BSONObj> metas;
  listMetadata(query, metas);
  if (metas.size() == 0)
    throw ltm_db::NoMatchingMessageException(coll_);
  mongo::BSONObj orig = metas.front();
  mongo::BSONObjBuilder new_meta_builder;

  std::set<std::string> fields;
  bson.getFieldNames(fields);

  BOOST_FOREACH (const string& f, fields) 
  {
    if ((f!="_id") && (f!="creation_time")) 
      new_meta_builder.append(BSON("$set" << BSON(f << bson.getField(f))).\
                              getField("$set"));
  }

  mongo::BSONObj new_meta = new_meta_builder.obj().copy();
  conn_->update(ns_, query, new_meta);
}

unsigned MongoMessageCollection::count()
{
  return conn_->count(ns_);
}

string MongoMessageCollection::collectionName() const
{
  return coll_;
}

} // namespace
