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

  Synchronizer for multiple Kinects.

  @author Brian Hamilton
  @author Richard Kelley

*/

#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H


// STD includes
#include <string>
#include <vector>
#include <deque>
    
//#include <iostream> //REMOVE
    
// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
    
// Boost Includes
#include <boost/thread.hpp>
#include <boost/cstdint.hpp> // for uint64_t_t
#include <boost/date_time.hpp> // for thread sleep
#include <boost/noncopyable.hpp> // to make class not copiable
#include <boost/signals2/mutex.hpp>

using boost::uint64_t;
using std::vector;
using std::deque;

namespace unr_rgbd {
  namespace multikinect {

class Synchronizer : private boost::noncopyable
{
  
 public:
  Synchronizer();
  ~Synchronizer();
  
  // Called to initalize number of streams to synchronize
  void initalize( unsigned numberStreams );

  // Called from manager callback
  void add( unsigned streamIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud );

  // Register callback

  // Accessors
  void setAgePenalty( double age ) {
    agePenalty_ = age;
  }
  double getAgePenalty() {
    return agePenalty_;
  }
  
  void setMaxDuration( uint64_t duration ) {
    maxDuration_ = duration;
  }
  uint64_t getMaxDuration() {
    return maxDuration_;
  }
  
  void setInterMessageBound( uint64_t bound ) {
    interMessageBound_ = bound;
  }
  uint64_t getInterMessageBound() {
    return interMessageBound_;
  }
  
 private:
  // Optional Peramiters
  double agePenalty_;
  uint64_t maxDuration_;
  uint64_t interMessageBound_;
  
  // Private Member Variables
  vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> candidate_;
  
  
  // Private Functions
  void checkInterMessageBound();
  void dequeDeleteFront( unsigned i);
  void dequeMoveFrontToPast( unsigned i);
  void makeCandidate();
  void recover( unsigned i , size_t numMessages );  // moves numMessages from the i'th past vector to the i'th deque
  void recover( unsigned i ); // moves everything from the i'th past vector to the i'th deque
  void recoverAndDelete( unsigned i ); //moves everthing from the past vector to the deque, and pop's the form of the deque
  void publishCandidate();
  void getCandidateStart( uint64_t *startIndex, uint64_t *startTime ) {
    return getCandidateBoundary( startIndex, startTime, false );
  }
  void getCandidateEnd( uint64_t *endIndex, uint64_t *endTime ) {
    return getCandidateBoundary( endIndex, endTime, true );
  }
  void getCandidateBoundary( uint64_t *index, uint64_t *time, bool ifEnd );

  unsigned getVirtualTime( unsigned i );
  void getVirtualCandidateStart( uint64_t *startIndex, uint64_t *startTime ) {
    return getVirtualCandidateBoundary( startIndex, startTime, false );
  }
  void getVirtualCandidateEnd( uint64_t *endIndex, uint64_t *endTime ) {
    return getVirtualCandidateBoundary( endIndex, endTime, true );
  }
  void getVirtualCandidateBoundary( uint64_t *index, uint64_t *time, bool ifEnd );
  void process();
};

  } // multikinect
} // unr_rgbd

#endif // SYNCHRONIZER_H
