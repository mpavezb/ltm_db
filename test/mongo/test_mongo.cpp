/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Test script for Mongo ros c++ interface
 *
 * \author Bhaskara Marthi
 */

// %Tag(CPP_CLIENT)%

#include "test_mongo_helpers.h"
#include <ltm_db/mongo/database_connection.h>
#include <gtest/gtest.h>

namespace gm=geometry_msgs;
using ltm_db::Metadata;
using ltm_db::Query;
using ltm_db::NoMatchingMessageException;
using std::vector;
using std::string;
using std::cout;

typedef ltm_db::MessageCollection<gm::Pose> PoseCollection;
typedef ltm_db::MessageWithMetadata<gm::Pose> PoseWithMetadata;
typedef PoseWithMetadata::ConstPtr PoseMetaPtr;

// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
Metadata::Ptr makeMetadata(PoseCollection coll, const gm::Pose& p, const string& n)
{
  Metadata::Ptr meta = coll.createMetadata();
  meta->append("x", p.position.x);
  meta->append("y", p.position.y);
  meta->append("name", n);
  return meta;
}

TEST(MongoRos, MongoRos)
{
  // Set up db
  ltm_db_mongo::MongoDatabaseConnection conn;
  conn.setParams("localhost", 27017, 60.0);
  conn.connect();

  // Clear existing data if any
  conn.dropDatabase("my_db");
  
  // Open the collection
  PoseCollection coll = conn.openCollection<gm::Pose>("my_db", "poses");

  // Arrange to index on metadata fields 'x' and 'name'
  //coll.ensureIndex("name");
  //coll.ensureIndex("x");

  // Add some poses and metadata
  const gm::Pose p1 = makePose(24, 42, 0);
  const gm::Pose p2 = makePose(10, 532, 3);
  const gm::Pose p3 = makePose(53, 22, 5);
  const gm::Pose p4 = makePose(22, -5, 33);
  coll.insert(p1, makeMetadata(coll, p1, "bar"));
  coll.insert(p2, makeMetadata(coll, p2, "baz"));
  coll.insert(p3, makeMetadata(coll, p3, "qux"));
  coll.insert(p1, makeMetadata(coll, p1, "oof"));
  coll.insert(p4, makeMetadata(coll, p4, "ooof"));
  EXPECT_EQ(5u, coll.count());

  // Simple query: find the pose with name 'qux' and return just its metadata
  // Since we're doing an equality check, we don't explicitly specify a predicate
  Query::Ptr q1 = coll.createQuery();
  q1->append("name", (std::string)"qux");
  vector<PoseMetaPtr> res = coll.queryList(q1, true);
  EXPECT_EQ(1u, res.size());
  EXPECT_EQ("qux", res[0]->lookupString("name"));
  EXPECT_DOUBLE_EQ(53, res[0]->lookupDouble("x"));
  
  // Set up query: position.x < 40 and position.y > 0.  Reverse order
  // by the "name" metadata field.  Also, here we pull the message itself, not
  // just the metadata.  Finally, we can't use the simplified construction
  // syntax here because it's too long
  Query::Ptr q2 = coll.createQuery();
  q2->appendLT("x", 40);
  q2->appendGT("y", 0);
  vector<PoseMetaPtr> poses = coll.queryList(q2, false, "name", false);
  
  // Verify poses. 
  EXPECT_EQ(3u, poses.size());
  EXPECT_EQ(p1, *poses[0]);
  EXPECT_EQ(p2, *poses[1]);
  EXPECT_EQ(p1, *poses[2]);

  EXPECT_EQ("oof", poses[0]->lookupString("name"));
  EXPECT_EQ("baz", poses[1]->lookupString("name"));
  EXPECT_EQ("bar", poses[2]->lookupString("name"));

  // Set up query to delete some poses.
  Query::Ptr q3 = coll.createQuery();
  q3->appendLT("y", 30);

  EXPECT_EQ(5u, coll.count());
  EXPECT_EQ(2u, coll.removeMessages(q3));
  EXPECT_EQ(3u, coll.count());

  // Test findOne
  Query::Ptr q4 = coll.createQuery();
  q4->append("name", (std::string)"bar");
  EXPECT_EQ(p1, *coll.findOne(q4, false));
  EXPECT_DOUBLE_EQ(24, coll.findOne(q4, true)->lookupDouble("x"));

  Query::Ptr q5 = coll.createQuery();
  q5->append("name", (std::string)"barbar");
  EXPECT_THROW(coll.findOne(q5, true), NoMatchingMessageException);
  EXPECT_THROW(coll.findOne(q5, false), NoMatchingMessageException);
  
  // Test update
  Metadata::Ptr m1 = coll.createMetadata();
  m1->append("name", (std::string)"barbar");
  coll.modifyMetadata(q4, m1);
  EXPECT_EQ(3u, coll.count());
  EXPECT_THROW(coll.findOne(q4, false), NoMatchingMessageException);
  EXPECT_EQ(p1, *coll.findOne(q5, false));

  // Check stored metadata
  EXPECT_EQ("geometry_msgs/Pose", conn.messageType("my_db", "poses"));
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "client_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
 
// %EndTag(CPP_CLIENT)%
