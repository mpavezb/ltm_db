/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Defines the MessageWithMetadata class as well as some helper functions
 * to create and manipulate Metadata objects
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_INTERFACE_MESSAGE_WITH_METADATA_H
#define LTM_DB_INTERFACE_MESSAGE_WITH_METADATA_H

#include <ltm_db/interface/metadata.h>

namespace ltm_db
{

/************************************************************
 * MessageWithMetadata
 ***********************************************************/

/// \brief Class that wraps (via inheritance) a ROS message type, together
/// with additional metadata (a yaml dictionary)
/// \tparam M the message type being wrapped
template<class M>
  struct MessageWithMetadata : public M
  {
  public:
    MessageWithMetadata(Metadata::ConstPtr metadata, const M& msg = M()) :
        M(msg), metadata_(metadata)
    {
    }

    MessageWithMetadata(const MessageWithMetadata& m) :
        M(m), metadata_(m.metadata_)
    {
    }

    MessageWithMetadata()
    {
    }

    Metadata::ConstPtr metadata_;

    std::string lookupString(const std::string& name) const
    {
      return metadata_->lookupString(name);
    }

    double lookupDouble(const std::string& name) const
    {
      return metadata_->lookupDouble(name);
    }

    int lookupInt(const std::string& name) const
    {
      return metadata_->lookupInt(name);
    }

    bool lookupBool(const std::string& name) const
    {
      return metadata_->lookupBool(name);
    }

    bool lookupField(const std::string& name) const
    {
      return metadata_->lookupField(name);
    }

    std::set<std::string> lookupFieldNames() const
    {
      return metadata_->lookupFieldNames();
    }

    typedef boost::shared_ptr<MessageWithMetadata<M> > Ptr;
    typedef boost::shared_ptr<const MessageWithMetadata<M> > ConstPtr;
  };

} // namespace

#endif // include guard
