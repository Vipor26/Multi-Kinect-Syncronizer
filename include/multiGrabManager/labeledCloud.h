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

  @author Brian Hamilton
*/

#ifndef LABELED_CLOUD_H
#define LABELED_CLOUD_H

// STD includes
#include <string>
#include <iostream>


// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace unr_rgbd {
  namespace multikinect {
  
    // Labeled cloud for keeping track of where the cloud came from
    // wraps the point cloud with a serialNumber
    struct LabeledCloud
    {
      // Public Functions
      LabeledCloud();
      LabeledCloud( const LabeledCloud &rhs );
      ~LabeledCloud();
      
      LabeledCloud& operator=(const LabeledCloud &rhs);
      
      // Public Data Members
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
      std::string serialNumber;
    };
    
    inline std::ostream& operator<<(std::ostream& s, const  LabeledCloud &data);
  
  } // multikinect
} // unr_rgbd

#endif // LABELED_CLOUD_H

