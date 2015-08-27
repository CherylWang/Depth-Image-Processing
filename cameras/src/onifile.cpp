/*
why 2015/8/28
*/

#include <dip/cameras/onifile.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace openni;

namespace dip {

OniFile::OniFile(const char *file_name) : enabled_(false) {
  // Initialize Variables
  for (int i = 0; i < SENSOR_TYPES; i++) {
    width_[i] = height_[i] = -1;
    fx_[i] = fy_[i] = -1.0f;
    running_[i] = false;
	count_[i] = 0;
  }

  // Initialize OpenNI
  OpenNI::initialize();
  printf("Initializing Camera\n%s\n", OpenNI::getExtendedError());

  // Initialize Camera
  initialize(file_name);
}



OniFile::~OniFile() {
  if (enabled_) {
    // Shutdown streams
    for (int i = 0; i < SENSOR_TYPES; i++) {
      if (stream_[i].isValid()) {
        if (running_[i])
          stream_[i].stop();
        stream_[i].destroy();
      }
    }
    device_.close();
    OpenNI::shutdown();
  }
}

int OniFile::Update(Depth *depth) {
  if (enabled_ && running_[DEPTH_SENSOR]) {
    if ((stream_[DEPTH_SENSOR].isValid())&&(count_[DEPTH_SENSOR]<total_frame)){
      stream_[DEPTH_SENSOR].readFrame(&frame_);
      if (frame_.isValid()) {
        const DepthPixel *pixels =
          static_cast<const DepthPixel*>(frame_.getData());

        // Copy depth frame.
        memcpy(depth, pixels, sizeof(Depth) *
               width_[DEPTH_SENSOR] * height_[DEPTH_SENSOR]);
		
		count_[DEPTH_SENSOR]++;
        return 0;
      }
    }
  }

  return -1;
}

int OniFile::Update(Color *color) {
  if (enabled_ && running_[COLOR_SENSOR]) {
    if ((stream_[COLOR_SENSOR].isValid())&&(count_[COLOR_SENSOR]<total_frame)){
      stream_[COLOR_SENSOR].readFrame(&frame_);

      if (frame_.isValid()) {
        const RGB888Pixel* pixels =
          static_cast<const RGB888Pixel*>(frame_.getData());

        // Copy color frame.
        memcpy(color, pixels, sizeof(Color) *
               width_[COLOR_SENSOR] * height_[COLOR_SENSOR]);
		count_[COLOR_SENSOR]++;
        return 0;
      }
    }
  }

  return -1;
}



int OniFile::start(int sensor) {
  if (enabled_ && !running_[sensor]) {
    if (stream_[sensor].isValid()) {
      stream_[sensor].start();
      running_[sensor] = true;

      return 0;
    }
  }

  return -1;
}

int OniFile::stop(int sensor) {
  if (enabled_ && running_[sensor]) {
    if (stream_[sensor].isValid()) {
      stream_[sensor].stop();
      running_[sensor] = false;

      return 0;
    }
  }

  return -1;
}

int OniFile::resolution(int sensor, int width, int height) {
  // Stop sensor's stream.
  if (!stop(sensor)) {
    // Modify video mode's resolution.
    VideoMode video_mode = stream_[sensor].getVideoMode();
    video_mode.setResolution(width, height);
    stream_[sensor].setVideoMode(video_mode);

    // Restart sensor's stream.
    if (!start(sensor)) {
      // Update dimensions and focal lengths.
      VideoMode video_mode = stream_[sensor].getVideoMode();

      width_[sensor] = video_mode.getResolutionX();
      height_[sensor] = video_mode.getResolutionY();

      float horizontal_fov = stream_[sensor].getHorizontalFieldOfView();
      float vertical_fov = stream_[sensor].getVerticalFieldOfView();

      fx_[sensor] = width_[sensor] / (2.0f * tan(horizontal_fov / 2.0f));
      fy_[sensor] = height_[sensor] / (2.0f * tan(vertical_fov / 2.0f));

      return 0;
    }
  }

  return -1;
}

void OniFile::initialize(const char *file_name) {
  // Open PrimeSense camera.
  if (device_.open(file_name) == STATUS_OK) {
    enabled_ = true;
    //why--------------------------------------------------------------------- s
    pController =device_.getPlaybackControl();
    pController->setSpeed(-1);
    //why---------------------------------------------------------------------  e
    // Create streams.
    SensorType sensor_types[SENSOR_TYPES];
    sensor_types[DEPTH_SENSOR] = SENSOR_DEPTH;
    sensor_types[COLOR_SENSOR] = SENSOR_COLOR;

    for (int i = 0; i < SENSOR_TYPES; i++) {
      if (stream_[i].create(device_, sensor_types[i]) == STATUS_OK) {
        // Disable Mirroring
        stream_[i].setMirroringEnabled(false);

        if(i==1){
         total_frame   = pController->getNumberOfFrames(stream_[i]);
        }//why--------------------------------------------------------------------

        // Start stream.
        stream_[i].start();
        running_[i] = true;

        // Grab dimensions of image.
        VideoMode video_mode = stream_[i].getVideoMode();

        width_[i] = video_mode.getResolutionX();
        height_[i] = video_mode.getResolutionY();

        // Compute Focal Lengths
        float horizontal_fov = stream_[i].getHorizontalFieldOfView();
        float vertical_fov = stream_[i].getVerticalFieldOfView();

        fx_[i] = width_[i] / (2.0f * tan(horizontal_fov / 2.0f));
        fy_[i] = height_[i] / (2.0f * tan(vertical_fov / 2.0f));
      }
      else {
        printf("Unable to Create Stream\n%s\n", OpenNI::getExtendedError());
      }
    }
    
  }
  else {
    printf("Unable to Open Camera\n%s\n", OpenNI::getExtendedError());
    OpenNI::shutdown();
  }
}

} // namespace dip
