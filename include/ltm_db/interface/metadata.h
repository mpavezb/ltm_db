/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Define a couple of classes for wrapping queries and metadata
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_INTERFACE_METADATA_H
#define LTM_DB_INTERFACE_METADATA_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <set>

namespace ltm_db
{

/// \brief Represents a query to the db
///
/// Usage:
/// q = Query().append("foo", 42).appendLT("bar", 24);
class Query
{
public:
  typedef boost::shared_ptr<Query> Ptr;
  typedef boost::shared_ptr<const Query> ConstPtr;

  virtual ~Query()
  {
  }
  virtual void append(const std::string& name, const std::string& val) = 0;
  virtual void append(const std::string& name, const double val) = 0;
  virtual void append(const std::string& name, const int val) = 0;
  virtual void append(const std::string& name, const bool val) = 0;
  virtual void appendLT(const std::string& name, const double val) = 0;
  virtual void appendLT(const std::string& name, const int val) = 0;
  virtual void appendLTE(const std::string& name, const double val) = 0;
  virtual void appendLTE(const std::string& name, const int val) = 0;
  virtual void appendGT(const std::string& name, const double val) = 0;
  virtual void appendGT(const std::string& name, const int val) = 0;
  virtual void appendGTE(const std::string& name, const double val) = 0;
  virtual void appendGTE(const std::string& name, const int val) = 0;
  virtual void appendRange(const std::string& name, const double lower, const double upper) = 0;
  virtual void appendRange(const std::string& name, const int lower, const int upper) = 0;
  virtual void appendRangeInclusive(const std::string& name, const double lower, const double upper) = 0;
  virtual void appendRangeInclusive(const std::string& name, const int lower, const int upper) = 0;
};

/// \brief Represents metadata attached to a message.
///
/// Usage:
/// m = Metadata().append("x", 24).append("name", "foo");
class Metadata
{
public:
  typedef boost::shared_ptr<Metadata> Ptr;
  typedef boost::shared_ptr<const Metadata> ConstPtr;

  virtual ~Metadata()
  {
  }
  virtual void append(const std::string& name, const std::string& val) = 0;
  virtual void append(const std::string& name, const double val) = 0;
  virtual void append(const std::string& name, const int val) = 0;
  virtual void append(const std::string& name, const bool val) = 0;
  virtual void append(const std::string& name, const std::vector<std::string>& val) = 0;
  virtual void append(const std::string& name, const std::vector<double>& val) = 0;
  virtual void append(const std::string& name, const std::vector<int>& val) = 0;
  virtual void append(const std::string& name, const std::vector<bool>& val) = 0;
  virtual void append(const std::string& name, const std::vector<uint32_t>& val) = 0;
  virtual void append(const std::string& name, const std::vector<float>& val) = 0;
  virtual std::string lookupString(const std::string& name) const = 0;
  virtual double lookupDouble(const std::string& name) const = 0;
  virtual int lookupInt(const std::string& name) const = 0;
  virtual bool lookupBool(const std::string& name) const = 0;
  virtual bool lookupField(const std::string& name) const = 0;
  virtual void lookupStringArray(const std::string& name, std::vector<std::string>& array) const = 0;
  virtual void lookupDoubleArray(const std::string& name, std::vector<double>& array) const = 0;
  virtual void lookupIntArray(const std::string& name, std::vector<int>& array) const = 0;
  virtual void lookupBoolArray(const std::string& name, std::vector<bool>& array) const = 0;
  virtual void lookupUInt32Array(const std::string& name, std::vector<uint32_t>& array) const = 0;
  virtual void lookupFloatArray(const std::string& name, std::vector<float>& array) const = 0;
  virtual std::set<std::string> lookupFieldNames() const = 0;
};

} // namespace

#endif // include guard
