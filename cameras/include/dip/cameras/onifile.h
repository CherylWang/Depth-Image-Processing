/*
Copyright (c) 2013-2015, Gregory P. Meyer
                         University of Illinois Board of Trustees
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Loads depth and color images from a HDF5 dump file.

#ifndef DIP_CAMERAS_ONIFILE_H
#define DIP_CAMERAS_ONIFILE_H

#include <dip/cameras/camera.h>
#include <dip/common/types.h>
#include <dip/common/macros.h>
#include <OpenNI.h>


namespace dip {

class OniFile : public Camera {
public:
  // Opens dump file.
  //  file_name - File name of .oni file
  OniFile(const char *file_name);
  ~OniFile();

  // Update depth image.
  //  depth - Buffer to hold the next depth image captured by the camera.
  //          The dimensions of the image should be the same as the dimensions
  //          returned by width() and height() functions.
  // Return zero when depth image is succesfully updated.
  int Update(Depth *depth);

  // Update color image.
  //  color - Buffer to hold the next color image captured by the camera.
  //          The dimensions of the image should be the same as the dimensions
  //          returned by width() and height() functions.
  // Return zero when color image is succesfully updated.
  int Update(Color *color);

  // Returns true if the camera was successfully enabled.
  bool enabled() const { return enabled_; }

  // Dimensions of depth/color images.
  int width(int sensor) const { return width_[sensor]; }
  int height(int sensor) const { return height_[sensor]; }

  // Focal length of depth/color images.
  float fx(int sensor) const { return fx_[sensor]; }
  float fy(int sensor) const { return fy_[sensor]; }

  // Request image resolution.
  int resolution(int sensor, int width, int height);


  // Start/Stop running the depth/color sensors.
  int start(int sensor);
  int stop(int sensor);

private:
  // Initialize PrimeSense camera.
  //  uri - String containing the URI of the camera.
  void initialize(const char *file_name);
 
  bool enabled_, running_[SENSOR_TYPES];
  int width_[SENSOR_TYPES], height_[SENSOR_TYPES];
  float fx_[SENSOR_TYPES], fy_[SENSOR_TYPES];

  
  // Handles PrimeSense sensor.
  openni::Device device_;
  openni::PlaybackControl* pController;
  // Manages depth and color streams.
  openni::VideoStream stream_[SENSOR_TYPES];

  // References to current depth and color frame.
  openni::VideoFrameRef frame_;

  int     total_frame;
  int count_[SENSOR_TYPES];
  DISALLOW_COPY_AND_ASSIGN(OniFile);

};

} // namespace dip

#endif // DIP_CAMERAS_ONIFILE_H
