/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Defines an iterator type over results of a query
 *
 * \author Bhaskara Marthi
 */

#ifndef LTM_DB_INTERFACE_QUERY_RESULTS_H
#define LTM_DB_INTERFACE_QUERY_RESULTS_H

#include <ltm_db/interface/message_with_metadata.h>
#include <ltm_db/interface/exceptions.h>
#include <boost/iterator/iterator_facade.hpp>

namespace ltm_db
{

class ResultIteratorHelper
{
public:
  virtual bool next() = 0;
  virtual bool hasData() const = 0;
  virtual Metadata::ConstPtr metadata() const = 0;
  virtual std::string message() const = 0;

  typedef boost::shared_ptr<ResultIteratorHelper> Ptr;
};

template<class M>
  class ResultIterator : public boost::iterator_facade<ResultIterator<M>, typename MessageWithMetadata<M>::ConstPtr,
      boost::single_pass_traversal_tag, typename MessageWithMetadata<M>::ConstPtr>
  {
  public:
    /// \brief Constructor
    ResultIterator(ResultIteratorHelper::Ptr results, bool metadata_only);

    /// \brief Copy constructor
    ResultIterator(const ResultIterator& rhs);

    /// \brief Constructor for past_the_end iterator
    ResultIterator();

    /// \brief Destructor
    ~ResultIterator();

    ResultIterator& operator=(const ResultIterator& other);

  private:
    friend class boost::iterator_core_access;

    // Member functions needed to be an iterator
    void increment();
    typename MessageWithMetadata<M>::ConstPtr dereference() const;
    bool equal(const ResultIterator<M>& other) const;

    ResultIteratorHelper::Ptr results_;
    const bool metadata_only_;
  };

template<class M>
  struct QueryResults
  {
    typedef std::pair<ResultIterator<M>, ResultIterator<M> > range_t;
  };

} // namespace

#include "impl/query_results_impl.hpp"

#endif // include guard
