/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Exceptions thrown by ltm_db
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_INTERFACE_EXCEPTIONS_H
#define LTM_DB_INTERFACE_EXCEPTIONS_H

#include <boost/format.hpp>
#include <stdexcept>
#include <string>

namespace ltm_db
{

using boost::format;

/// A base class for all ltm_db exceptions; provides a handy boost::format parent constructor
class WarehouseRosException : public std::runtime_error
{
public:
  WarehouseRosException(const format& error_string) :
      std::runtime_error(error_string.str())
  {
  }
  ;
  WarehouseRosException(const char* str) :
      std::runtime_error(str)
  {
  }
  ;
};

/// \brief Couldn't find matching message in collection
struct NoMatchingMessageException : public WarehouseRosException
{
  NoMatchingMessageException(const std::string& coll) :
      WarehouseRosException(format("Couldn't find message in %1% matching query") % coll)
  {
  }
};

/// \brief Couldn't connect to database
struct DbConnectException : public WarehouseRosException
{
  DbConnectException(const std::string& failure) :
      WarehouseRosException(format("Not connected to the database. %1%") % failure)
  {
  }
};

/// \brief Different md5 sum for messages
struct Md5SumException : public WarehouseRosException
{
  Md5SumException(const std::string& failure) :
      WarehouseRosException(
          format(
              "The md5 sum for the ROS messages saved in the database differs from that of the compiled message. %1%")
              % failure)
  {
  }
};

} // namespace

#endif // include guard
