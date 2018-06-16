/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Implements TransformCollection class
 *
 * \author Bhaskara Marthi
 */

#include <ltm_db/interface/transform_collection.h>
#include <boost/foreach.hpp>

namespace ltm_db
{

tf::StampedTransform TransformCollection::lookupTransform(const std::string& target, const std::string& src,
                                                          const double t) const
{
  // Query all transforms between t-search_back_ and t+search_forward_
  Query::Ptr q = coll_.createQuery();
  q->appendRangeInclusive("stamp", t - search_back_, t + search_forward_);

  // Iterate over the messages and add them to a Transformer
  tf::Transformer buffer(true, ros::Duration(search_back_ + search_forward_ * 1.1));
  typename QueryResults<tf::tfMessage>::range_t res = coll_.query(q);
  for (ResultIterator<tf::tfMessage> it = res.first; it != res.second; ++it)
  {
    BOOST_FOREACH (const geometry_msgs::TransformStamped& trans, (*it)->transforms)
    {
      const geometry_msgs::Vector3& v = trans.transform.translation;
      const geometry_msgs::Quaternion& q = trans.transform.rotation;
      const std_msgs::Header& h = trans.header;
      const tf::Transform tr(tf::Quaternion(q.x, q.y, q.z, q.w), tf::Vector3(v.x, v.y, v.z));
      const tf::StampedTransform t(tr, h.stamp, h.frame_id, trans.child_frame_id);
      const bool ok = buffer.setTransform(t);
      ROS_ASSERT_MSG(ok, "Tf setTransform returned false for transform from %s "
                     "to %s at %.4f",
                     trans.child_frame_id.c_str(), h.frame_id.c_str(), h.stamp.toSec());
    }
  }
  tf::StampedTransform result;
  buffer.lookupTransform(target, src, ros::Time(t), result); // Can throw
  return result;
}

void TransformCollection::putTransform(tf::StampedTransform)
{

}

} // namespace
