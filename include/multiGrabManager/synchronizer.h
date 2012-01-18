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
  @author Tina Nye

*/

#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H

// STD includes
#include <string>
#include <vector>
#include <deque>
#include <limits>
    
//#include <iostream> //REMOVE
    
// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
    
// Boost Includes
#include <boost/thread.hpp>
#include <boost/cstdint.hpp> // for uint64_t
#include <boost/date_time.hpp> // for thread sleep
#include <boost/noncopyable.hpp> // to make class not copiable
#include <boost/signals2.hpp>

using boost::uint64_t;
using std::vector;
using std::deque;

namespace unr_rgbd {
  namespace multikinect {

    typedef uint64_t TimeStamp;
    typedef uint64_t Duration;
  
    class Synchronizer : private boost::noncopyable
    {
     public:
      Synchronizer();
      ~Synchronizer();
      
      // Called to initalize number of streams to synchronize
      void initalize( unsigned numberStreams, unsigned queueSize = 5);

      // Called from manager callback
      void add( unsigned streamIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud );

      // Register callback
      void registerCallback( boost::function<void 
             (vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>&)> f ); 
      
      // Accessors
      void setAgePenalty( double age ) {
        agePenalty_ = age;
      }
      double getAgePenalty() {
        return agePenalty_;
      }
      
      void setMaxDuration( Duration duration ) {
        maxDuration_ = duration;
      }
      Duration getMaxDuration() {
        return maxDuration_;
      }
      
      void setInterMessageBound( unsigned i, Duration bound ) {
        interMessageBounds_[i] = bound;
      }
      uint64_t getInterMessageBound(unsigned i) {
        return interMessageBounds_[i];
      }
      
     private:
      
      // Optional Peramiters
      double agePenalty_;
      Duration maxDuration_;
      vector<Duration> interMessageBounds_;

      // Signal stuff
      boost::signals2::signal<void ( vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& )> 
                                                                            mainSignal_;
      boost::signals2::connection mainConnection_;

      // Private Member Variables
      bool hasPivot_;
      unsigned queueSize_;
      unsigned pivotIndex_;
      unsigned numStreams_;
      unsigned numNonEmptyDeques_;
      
      boost::mutex dataMutex_;
      
      TimeStamp pivotTime_;
      TimeStamp candidateEnd_;
      TimeStamp candidateStart_;
      
      vector<bool> hasDroppedMessages_;
      vector<bool> warnedAboutIncorrectBounds_;
      vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> candidate_;
      vector<deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > deques_;
      vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > histories_;

      // Private Functions
      void checkInterMessageBound(unsigned i);
      void dequeDeleteFront( unsigned i);
      void dequeMoveFrontToPast( unsigned i);
      void makeCandidate();

      // moves numMessages from the i'th past vector to the i'th deque
      void recover( unsigned i , size_t numMessages );

      // moves everything from the i'th past vector to the i'th deque
      void recover( unsigned i ); 

      //moves everthing from the past vector to the deque, and pop's the form of the deque
      void recoverAndDelete( unsigned i ); 
      void publishCandidate();
      void getCandidateStart( unsigned *startIndex, TimeStamp *startTime ) {
        return getCandidateBoundary( startIndex, startTime, false );
      }
      void getCandidateEnd( unsigned *endIndex, TimeStamp *endTime ) {
        return getCandidateBoundary( endIndex, endTime, true );
      }
      void getCandidateBoundary( unsigned *index, TimeStamp *time, bool ifEnd );

      TimeStamp getVirtualTime( unsigned i );
      void getVirtualCandidateStart( unsigned *startIndex, TimeStamp *startTime ) {
        return getVirtualCandidateBoundary( startIndex, startTime, false );
      }
      void getVirtualCandidateEnd( unsigned *endIndex, TimeStamp *endTime ) {
        return getVirtualCandidateBoundary( endIndex, endTime, true );
      }
      void getVirtualCandidateBoundary( unsigned *index, TimeStamp *time, bool ifEnd );
      void process();
    };

  } // multikinect
} // unr_rgbd

#endif // SYNCHRONIZER_H
