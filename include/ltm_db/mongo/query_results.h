/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Implementation of ltm_db::ResultIteratorHelper for mongo queries
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_MONGO_QUERY_RESULTS_H
#define LTM_DB_MONGO_QUERY_RESULTS_H

#include <ltm_db/mongo/metadata.h>
#include <ltm_db/interface/query_results.h>
#include <boost/optional.hpp>

namespace ltm_db_mongo
{

// To avoid some const-correctness issues we wrap Mongo's returned auto_ptr in
// another pointer
typedef std::auto_ptr<mongo::DBClientCursor> Cursor;
typedef boost::shared_ptr<Cursor> CursorPtr;

class MongoResultIterator : public ltm_db::ResultIteratorHelper
{
public:
  MongoResultIterator(boost::shared_ptr<mongo::DBClientConnection> conn,
                      boost::shared_ptr<mongo::GridFS> gfs,
                      const std::string& ns,
                      const mongo::Query& query);
  bool next();
  bool hasData() const;
  ltm_db::Metadata::ConstPtr metadata() const;
  std::string message() const;
  mongo::BSONObj metadataRaw() const;

private:
  CursorPtr cursor_;
  boost::optional<mongo::BSONObj> next_;
  boost::shared_ptr<mongo::GridFS> gfs_;
};

} // namespace

#endif // include guard
