 /*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Implementation of MongoResultIterator.
 *
 * \author Bhaskara Marthi
 */

#include <ltm_db/mongo/query_results.h>

namespace ltm_db_mongo
{

MongoResultIterator::MongoResultIterator(boost::shared_ptr<mongo::DBClientConnection> conn,
                                         boost::shared_ptr<mongo::GridFS> gfs,
                                         const std::string& ns,
                                         const mongo::Query& query) :
  cursor_(new Cursor(conn->query(ns, query))),
  gfs_(gfs)
{
  if ((*cursor_)->more())
    next_ = (*cursor_)->nextSafe();
}

bool MongoResultIterator::next()
{
  ROS_ASSERT (next_);
  if ((*cursor_)->more())\
  {
    next_ = (*cursor_)->nextSafe();
    return true;
  }
  else
  {
    next_.reset();
    return false;
  }
}

bool MongoResultIterator::hasData() const
{
  return (bool)next_;
}

ltm_db::Metadata::ConstPtr MongoResultIterator::metadata() const
{
  ROS_ASSERT(next_);
  return typename ltm_db::Metadata::ConstPtr(new MongoMetadata(next_->copy()));
}

std::string MongoResultIterator::message() const
{
  mongo::OID blob_id;
  (*next_)["blob_id"].Val(blob_id);
  mongo::BSONObj q = BSON ("_id" << blob_id);
  mongo::GridFile f = gfs_->findFile(q);
  ROS_ASSERT(f.exists());
  std::stringstream ss (std::ios_base::out);
  f.write(ss);
  return ss.str();
}

mongo::BSONObj MongoResultIterator::metadataRaw() const
{
  ROS_ASSERT(next_);
  return next_->copy();
}

} // namespace
