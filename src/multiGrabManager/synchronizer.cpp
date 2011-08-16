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
  @author Tina Nye
  @author Katie Browne
*/

#include <synchronizer.h>

namespace unr_rgbd {
  namespace multikinect {
  
    Synchronizer::Synchronizer(unsigned queueSize)
      : queueSize_(queueSize)
      , numNonEmptyDeques_(0)
      , maxDuration_(std::numeric_limits<boost::uint64_t>::max())
      , agePenalty_(0.1)
      , pivotIndex_(0)
      , hasPivot_(false)
      , numStreams_(0)
      , pivotTime_(0)
      , candidateEnd_(0)
      , candidateStart_(0)
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
      // TODO CHECK ME
      if( numStreams_ != 0 )  {
        // Clear out vectors
        hasDroppedMessages_.clear();
        interMessageBounds_.clear();
        warnedAboutIncorrectBounds_.clear();
        candidate_.clear();
        deques_.clear();
        histories_.clear();
        
        numNonEmptyDeques_ = 0;
        pivotIndex_ = 0;
        hasPivot_ = false;
        pivotTime_ = 0;
        candidateEnd_ = 0;
        candidateStart_ = 0;
      }
      hasDroppedMessages_.resize(numberStreams);
      interMessageBounds_.resize(numberStreams);
      warnedAboutIncorrectBounds_.resize(numberStreams);
      candidate_.resize(numberStreams);
      deques_.resize(numberStreams);
      histories_.resize(numberStreams);
      
      for( unsigned i=0; i< numberStreams; i++ )
      {
        interMessageBounds_[i] = 0;
        hasDroppedMessages_[i] = false;
        warnedAboutIncorrectBounds_[i] = false;
      }
      numStreams_ = numberStreams;
    }
    
    // Called from manager callback
    void Synchronizer::add( unsigned streamIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud )
    {
      boost::mutex::scoped_lock lock(dataMutex_);

      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *d = &deques_[streamIndex];
      d->push_back(cloud);
      if (d->size() == 1) {
        ++numNonEmptyDeques_;
	      if (numNonEmptyDeques_ == numStreams_) {
	        process();
	      }
      } else {
	      checkInterMessageBound(streamIndex);
      }

      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *v = &histories_[streamIndex];
      
      if (d->size() + v->size() > queueSize_) {
	      numNonEmptyDeques_ = 0;
	      for (unsigned i = 0; i < numStreams_; ++i) {
	        recover(i);
	      }

	      if (!d->empty()) {
	        return;
	      }

	      d->pop_front();
	      hasDroppedMessages_[streamIndex] = true;
	
	      if (hasPivot_ == true) {
	        hasPivot_ = false;
	        process();
	      }
      }
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

      if (d->empty()) {
	      return;
      }

      TimeStamp msg_time = d->back()->header.stamp;
      TimeStamp previous_msg_time;

      if (d->size() == 1) {
	      if (v->empty()) {
	        return;
	      }
	      previous_msg_time = v->back()->header.stamp;
      } else { 
      	previous_msg_time = (*d)[d->size()-2]->header.stamp;
      }

      if (msg_time < previous_msg_time) {
	      // TODO insert warning - messages out of order.
	      warnedAboutIncorrectBounds_[i] = true;
      } else if ((msg_time - previous_msg_time) < interMessageBounds_[i]) {
	      // TODO insert warning - time bound not respected.
	      warnedAboutIncorrectBounds_[i] = true;
      }
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
      for (unsigned i = 0; i < numStreams_; ++i) {
	      candidate_[i] = deques_[i].front();
	      histories_[i].clear();
      }
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
      //TODO: Signal Callback with candidate
      // candidate_
      
      hasPivot_ = false; //TODO CHECKME
      
       numNonEmptyDeques_ = 0;
       for( unsigned i = 0; i< numStreams_; i++ ) {
         recoverAndDelete(i);
       }
    }
    
    void Synchronizer::getCandidateBoundary( unsigned *index, TimeStamp *time, bool ifEnd )
    {
      //TODO Check me
      TimeStamp testTime;
      (*time) = deques_[0].front()->header.stamp;
      (*index) = 0;
      for(unsigned i=1; i<numStreams_; i++)
      {
        testTime = deques_[i].front()->header.stamp;
        if( ( testTime < (*time) ) ^ ifEnd )
        {
          (*time) = testTime;
          (*index) = i;
        }
      }
    }
    
    TimeStamp Synchronizer::getVirtualTime( unsigned i )
    {
      if( hasPivot_ == false )  { //TODO check me
        return 0;
      }
      
      std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* d = &deques_[i];
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* v = &histories_[i];
      
      if( d->empty() )
      {
        if( v->empty() )  { //TODO: check me
          return 0;
        }
        
        TimeStamp lastMsgTime = v->back()->header.stamp;
        TimeStamp msgTimeLowerBound = lastMsgTime + interMessageBounds_[i];
        if( msgTimeLowerBound > pivotTime_ )  {
          return msgTimeLowerBound;
        }
        // else return pivit time
        return pivotTime_;
      }
      TimeStamp currentMsgTime = d->front()->header.stamp;
      return currentMsgTime;
    }
    
    void Synchronizer::getVirtualCandidateBoundary( unsigned *index, TimeStamp *time, bool ifEnd )
    {
      TimeStamp testTime;
      (*time) = getVirtualTime(0);
      (*index) = 0;
      for( unsigned i=1; i< numStreams_; i++ )  {
        testTime = getVirtualTime(i);
        if( ( testTime < (*time) ) ^ ifEnd )  {
          (*time) = testTime;
          (*index) = i;
        }
      }
    }
    
    void Synchronizer::process()
    {
      unsigned  endIndex = 0, startIndex = 0;
      TimeStamp endTime  = 0, startTime  = 0;
      
      while( numNonEmptyDeques_ == numStreams_ )
      {
        getCandidateStart( &startIndex, &startTime );
        getCandidateEnd( &endIndex, &startTime );
        
        for( unsigned i=0; i<numStreams_; i++ )
        {
          if( i != endIndex ) {
            hasDroppedMessages_[i] = false;
          }
        }
        
        if( hasPivot_ == false )
        {
          if( endTime - startTime > maxDuration_ )  {
            dequeDeleteFront(startIndex);
            continue;
          }
          if( hasDroppedMessages_[endIndex] == true ) {
            dequeDeleteFront(startIndex);
            continue;
          }
          //Valid Candidate Set found
          makeCandidate();
          candidateStart_ = startTime;
          candidateEnd_ = endTime;
          pivotIndex_ = endIndex;
          pivotTime_ = endTime;
          dequeMoveFrontToPast( startIndex );
        }
        else  { //( hasPivot_ == true )
          if((endTime - candidateEnd_) * ( 1 + agePenalty_ ) >= (startTime - candidateStart_)) {
            dequeMoveFrontToPast( startIndex );            
          }
          else {
            makeCandidate();
            candidateStart_ = startTime;
            candidateEnd_ = endTime;
            // Keep the same pivit time and index // TODO why?
            dequeMoveFrontToPast( startIndex );
          }
        }
        if( hasPivot_ == false )  {
          // TODO  cerr <<
          return;
        }
        if( startIndex == pivotIndex_ ) {
          publishCandidate();
        }
        else if((endTime - candidateEnd_) * (1 +  agePenalty_ ) >= (pivotTime_ - candidateStart_)) {
          publishCandidate();
        }
        else if( numNonEmptyDeques_ < numStreams_ )
        {
          vector< unsigned > numVirtualMoves( numStreams_, 0 );
          while( true )
          {
            unsigned  virtualEndIndex = 0, virtualStartIndex = 0;
            TimeStamp virtualEndTime  = 0, virtualStartTime  = 0;
            
            
            getVirtualCandidateStart( &virtualStartIndex, &virtualStartTime );
            getVirtualCandidateEnd( &virtualEndIndex, &virtualEndTime );
            if((virtualEndTime - candidateEnd_) * (1 +  agePenalty_ ) >= (pivotTime_ - candidateStart_))
            {
              publishCandidate();
              break;
            }
            if((virtualEndTime - candidateEnd_) * (1 +  agePenalty_ ) < (virtualStartTime - candidateStart_))
            {
              numNonEmptyDeques_ = 0;
            
              for( unsigned i=0; i<numStreams_; i++ )
              {
                recover(i, numVirtualMoves[i]);
              }
              break;
            }
            if( virtualStartIndex == pivotIndex_ )  {
              // TODO cerr
              return;
            }
            if( virtualStartTime == pivotTime_ )  {
              // TODO cerr
              return;
            }
            dequeMoveFrontToPast( virtualStartIndex );
            numVirtualMoves[virtualStartIndex]++;
          } // while 1 
        }
      }
    }
    
  } // multikinect
} // unr_rgbd
