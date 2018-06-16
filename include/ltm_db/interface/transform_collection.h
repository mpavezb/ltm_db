/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Defines the TransformCollection class
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_INTERFACE_TF_COLLECTION_H
#define LTM_DB_INTERFACE_TF_COLLECTION_H

#include <ltm_db/interface/message_collection.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

namespace ltm_db
{

/// This abstract base class just makes it easier to write code that works for
/// both TransformCollection and regular tf TransformListener objects
class TransformSource
{
public:
  /// Get the transform between two frames at a given timepoint.  Can throw
  /// all the usual tf exceptions if the transform is unavailable.
  virtual tf::StampedTransform lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                               double t) const = 0;
};

/// The setup is that you have a db containing a collection with tf messages,
/// in which each message has a metadata field named 'stamp', that equals the
/// tf timestamp (this could be generated, e.g., with bag_to_db followed by
/// add_metadata).  
/// Given such a collection, this class allows querying for a transform as
/// with tf::TransformListener::lookupTransform, except this is deterministic
/// with no dependency on network state or message queues.
class TransformCollection : public TransformSource
{
public:
  TransformCollection(MessageCollection<tf::tfMessage> &coll, const double search_back = 10.0,
                      const double search_forward = 1.0) :
      TransformSource(), coll_(coll), search_back_(search_back), search_forward_(search_forward)
  {
  }

  /// Get the transform between two frames at a given timepoint.  Can throw
  /// all the exceptions tf::lookupTransform can.
  virtual tf::StampedTransform lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                               double t) const;

  /// Put the transform into the collection.
  void putTransform(tf::StampedTransform);

private:
  MessageCollection<tf::tfMessage> coll_;
  double search_back_;
  double search_forward_;
};

/// This wraps a tf transform listener so it can be used interchangeably
/// with a TransformCollection.  
class LiveTransformSource : public TransformSource
{
public:
  /// \param timeout: Maximum timeout
  ///
  /// ros::init must be called before creating an instance
  LiveTransformSource(double timeout = 0) :
      TransformSource(), tf_(new tf::TransformListener()), timeout_(timeout)
  {
  }

  /// Will return the transform if it becomes available before the timeout 
  /// expires, else throw a tf exception
  virtual tf::StampedTransform lookupTransform(const std::string& target, const std::string& source, double t) const
  {
    ros::Time tm(t);
    tf_->waitForTransform(target, source, tm, ros::Duration(timeout_));
    tf::StampedTransform trans;
    tf_->lookupTransform(target, source, tm, trans); // Can throw
    return trans;
  }

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> tf_;
  ros::Duration timeout_;
};

} // namespace

#endif // include guard
