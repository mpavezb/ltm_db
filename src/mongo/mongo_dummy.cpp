/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

#include <ltm_db/mongo/metadata.h>

// add this dummy function so the .so file copies everything we need from the
// libmongoclient.a file at link time. We need this because Ubuntu does not install
// a .so file for libmongoclient and the wrappers we have in this lib are templated.
// make this function globally accessible so strip --strip-unneeded does not remove symbols
void _thisFunctionShouldNeverBeCalled_MakeWarehouseROSMongoIncludeTheSymbolsWeNeed_(void)
{
  mongo::DBClientConnection *conn = new mongo::DBClientConnection();
  mongo::GridFS *gfs = new mongo::GridFS(*conn, "");
  mongo::BSONObj q;
  mongo::GridFile f = gfs->findFile(q);
  f.write(std::cout);
  gfs->removeFile("");
  q = gfs->storeFile(NULL, 0, "");
  mongo::LT;
  mongo::GT;
  mongo::LTE;
  mongo::GTE;
  delete gfs;
  delete conn;
}
