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


Synchronizer::Synchronizer()
{
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
  
  
// Private Functions
void Synchronizer::checkInterMessageBound()
{
}

void Synchronizer::dequeDeleteFront( unsigned i)
{
}

void Synchronizer::dequeMoveFrontToPast( unsigned i)
{
}

void Synchronizer::makeCandidate()
{
}

// moves numMessages from the i'th past vector to the i'th deque
void Synchronizer::recover( unsigned i , size_t numMessages )
{
}
  
// moves everything from the i'th past vector to the i'th deque
void Synchronizer::recover( unsigned i )
{
}

//moves everthing from the past vector to the deque, and pop's the form of the deque
void Synchronizer::recoverAndDelete( unsigned i )
{
}

void Synchronizer::publishCandidate()
{
}

void Synchronizer::getCandidateBoundary( uint64_t *index, uint64_t *time, bool ifEnd )
{
}

unsigned Synchronizer::getVirtualTime( unsigned i )
{
}

void Synchronizer::getVirtualCandidateBoundary( uint64_t *index, uint64_t *time, bool ifEnd )
{
}

void Synchronizer::process()
{
}


  // Optional Peramiters
  //double agePenalty_;
  //uint64_t maxDuration_;
  //uint64_t interMessageBound_;
  
  // Private Member Variables
  //vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr &> candidate_;

  } // multikinect
} // unr_rgbd
