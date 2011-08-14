/*
  Copyright (c) 2011, Board of Regents, NSHE, obo UNR 
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  Implementation of synchronizer class.

  @author Brian Hamilton
  @author Richard Kelley
*/

#include <synchronizer.h>

namespace unr_rgbd {
  namespace multikinect {


    Synchronizer::Synchronizer(unsigned queue_size)
      : queue_size_(queue_size)
      , numNonEmptyDeques_(0)
      , maxDuration_(std::numeric_limits<boost::uint64_t>::max())
      , agePenalty_(0.1)
      , pivotIndex_(0)
      , hasPivot_(false)
      , numStreams_(0)
    {
      // TODO FINISH INITIALIZERS
      // TODO assert queueSize > 0
    }
    Synchronizer::~Synchronizer()
    {
    }
    
    // Called to initalize number of streams to synchronize
    void Synchronizer::initalize( unsigned numberStreams )
    {
    }
    
    // Called from manager callback
    void Synchronizer::add( unsigned streamIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud )
    {
    }
    
    // Register callback
    void Synchronizer::registerCallback( boost::function<void (vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>)> f )
    {
      
    }
    
    // Private Functions
    void Synchronizer::checkInterMessageBound( unsigned i )
    {
      if( ( i > numStreams_ ) || ( warnedAboutIncorrectBounds_[i] ) ) {
	return;
      }

      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *d = &deques_[i];
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *v = &histories_[i];

    }
    
    void Synchronizer::dequeDeleteFront( unsigned i)
    {
      if (i > deques_.size()) {
	return;
      }
      
      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *d =  &deques_[i];
      
      if (d->empty()) {
	return;
      }
      
      d->pop_front();
      
      if (d->empty()) {
	--numNonEmptyDeques_;
      }      
    }
    
    void Synchronizer::dequeMoveFrontToPast( unsigned i)
    {
      if (i > deques_.size()) {
	return;
      }
      
      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *d = &deques_[i];
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *v = &histories_[i];
      
      if (d->empty()) {
	return;
      }
      
      v->push_back(d->front());
      d->pop_front();
      if (d->empty()) {
	--numNonEmptyDeques_;
      }
    }
    
    void Synchronizer::makeCandidate()
    {
      
      // create candidate tuple
      
      // delete all past messages
      
    }
    
    // moves numMessages from the i'th past vector to the i'th deque
    // precondition: i-th deque is nonempty.
    void Synchronizer::recover( unsigned i , size_t numMessages )
    {
      if (i > deques_.size()) {
	return;
      }
      
      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *d = &deques_[i];
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *v = &histories_[i];
      
      if (numMessages > v->size()) {
	return;
      }
      
      while(numMessages > 0) {
	d->push_front(v->back());
	v->pop_back();
	--numMessages;
      }
      
      if (!d->empty()) {
	++numNonEmptyDeques_;
      }
    }
    
    // moves everything from the i'th past vector to the i'th deque
    // precondition: i-th deque is nonempty.
    void Synchronizer::recover( unsigned i )
    {
      
      if (i > deques_.size()) {
	return;
      }
      
      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* d = &deques_[i];
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* v = &histories_[i];
      
      while(!v->empty()) {
	d->push_front(v->back());
	v->pop_back();
      }
      
      if (!d->empty()) {
	++numNonEmptyDeques_;
      }      
    }
    
    //moves everthing from the past vector to the deque, and pop's the form of the deque
    // precondition: i-th deque is nonempty.
    void Synchronizer::recoverAndDelete( unsigned i )
    {
      if (i > deques_.size()) {
	return;
      }
      
      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* d = &deques_[i];
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* v = &histories_[i];
      
      while(!v->empty()) {
	d->push_front(v->back());
	v->pop_back();
      }
      
      if (d->empty()) {
	return;
      }
      
      d->pop_front();
      if(!d->empty()) {
	++numNonEmptyDeques_;
      }
      
    }
    void Synchronizer::publishCandidate()
    {
      
    }
    
    void Synchronizer::getCandidateBoundary( unsigned *index, TimeStamp *time, bool ifEnd )
    {
      
    }
    
    unsigned Synchronizer::getVirtualTime( unsigned i )
    {
      
    }
    
    void Synchronizer::getVirtualCandidateBoundary( unsigned *index, TimeStamp *time, bool ifEnd )
    {
      
    }
    
    void Synchronizer::process()
    {
      
    }
    
  } // multikinect
} // unr_rgbd
