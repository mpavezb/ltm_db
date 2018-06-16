/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

/**
 * \file 
 * 
 * Template implementation for ResultIterator.
 * Only to be included from query_results.h
 *
 * \author Bhaskara Marthi
 */

namespace ltm_db
{

template<class M>
  ResultIterator<M>::ResultIterator(ResultIteratorHelper::Ptr results, bool metadata_only) :
      results_(results), metadata_only_(metadata_only)
  {
    if (!results_->hasData())
      results_.reset();
  }

template<class M>
  ResultIterator<M>::ResultIterator(const ResultIterator<M>& other) :
      results_(other.results_), metadata_only_(other.metadata_only_)
  {
  }

template<class M>
  ResultIterator<M>::ResultIterator() :
      metadata_only_(false)
  {
  }

template<class M>
  ResultIterator<M>::~ResultIterator()
  {
  }

template<class M>
  ResultIterator<M>& ResultIterator<M>::operator=(const ResultIterator& other)
  {
    results_ = other.results_;
    metadata_only_ = other.metadata_only_;
    return *this;
  }

template<class M>
  void ResultIterator<M>::increment()
  {
    if (!results_->next())
    {
      results_.reset();
    }
  }

template<class M>
  typename MessageWithMetadata<M>::ConstPtr ResultIterator<M>::dereference() const
  {
    ROS_ASSERT(results_);

    typename MessageWithMetadata<M>::Ptr msg(new MessageWithMetadata<M>(results_->metadata()));
    if (!metadata_only_)
    {
      std::string str = results_->message();
      uint8_t* buf = (uint8_t*)str.c_str();
      ros::serialization::IStream istream(buf, str.size());
      ros::serialization::Serializer<M>::read(istream, *msg);
    }
    return msg;
  }

template<class M>
  bool ResultIterator<M>::equal(const ResultIterator<M>& other) const
  {
    // Incomplete, the only case we care about is whether iter is at the end
    return (!results_ && !other.results_);
  }

} // namespace
