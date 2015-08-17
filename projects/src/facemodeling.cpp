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

#include <dip/projects/facemodeling.h>

#include <stdio.h>

using namespace Eigen;

namespace dip {

FaceModeling::FaceModeling(int width, int height, float fx, float fy,
                           float cx, float cy) :
                           min_weight_(0.0f), width_(width), height_(height),
                           fx_(fx), fy_(fy), cx_(cx), cy_(cy),
                           initial_frame_(true), failed_frames_(0) {

//why-----------------------------------1s Allocate could umage on CPU

denoised_depth_c = new Depth[width_*height_];

count = 0;
count_num = 0;

cloud_1.x = new float[width_*height_];
cloud_1.y = new float[width_*height_];
cloud_1.z = new float[width_*height_];


cloud_2.width = width_;
cloud_2.height = height_;
cloud_2.is_dense = false;
cloud_2.points.resize(cloud_2.width*cloud_2.height);



//why-----------------------------------1e



  // Allocate depth image on the CPU.
  depth_ = new Depth[width_ * height_];

  // Allocate depth images on the GPU.
  Allocate((void**)&segmented_depth_, sizeof(Depth) * width_ * height_);
  Allocate((void**)&filtered_depth_, sizeof(Depth) * width_ * height_);
  Allocate((void**)&denoised_depth_, sizeof(Depth) * width_ * height_);

  // Allocate pyramids on the GPU.
  for (int i = 0 ; i < kPyramidLevels; i++) {
    Allocate((void**)&(depth_pyramid_[i]), sizeof(Depth) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));

    Allocate((void**)&(vertices_[i].x), sizeof(float) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));
    Allocate((void**)&(vertices_[i].y), sizeof(float) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));
    Allocate((void**)&(vertices_[i].z), sizeof(float) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));

    Allocate((void**)&(normals_[i].x), sizeof(float) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));
    Allocate((void**)&(normals_[i].y), sizeof(float) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));
    Allocate((void**)&(normals_[i].z), sizeof(float) *
             (width_ >> (i * kDownsampleFactor)) *
             (height_ >> (i * kDownsampleFactor)));
  }

  Allocate((void**)&(model_vertices_.x), sizeof(float) * width_ * height_);
  Allocate((void**)&(model_vertices_.y), sizeof(float) * width_ * height_);
  Allocate((void**)&(model_vertices_.z), sizeof(float) * width_ * height_);

  Allocate((void**)&(model_normals_.x), sizeof(float) * width_ * height_);
  Allocate((void**)&(model_normals_.y), sizeof(float) * width_ * height_);
  Allocate((void**)&(model_normals_.z), sizeof(float) * width_ * height_);

  // Allocate the volume on the GPU and
  // set the value of each voxel to zero.
  Allocate((void**)&volume_, sizeof(Voxel) *
           kVolumeSize * kVolumeSize * kVolumeSize);
  Clear((void*)volume_, sizeof(Voxel) *
        kVolumeSize * kVolumeSize * kVolumeSize);

  // Allocate normal map on GPU.
  Allocate((void**)&normal_map_, sizeof(Color) * width_ * height_);

  // Initialize rigid body transformation to the identity matrix.
  transformation_.setIdentity();
}

FaceModeling::~FaceModeling() {

 //------------------------------------------why ~


 delete [] denoised_depth_c;
 delete [] cloud_1.x;
 delete [] cloud_1.y;
 delete [] cloud_1.z;

 //delete [] vertices_C;
 //Deallocate((void*)(vertices_G));


 //-------------------------------------------why ~

  delete [] depth_;

  Deallocate((void*)(segmented_depth_));
  Deallocate((void*)(filtered_depth_));
  Deallocate((void*)(denoised_depth_));

  for (int i = 0 ; i < kPyramidLevels; i++) {
    Deallocate((void*)depth_pyramid_[i]);

    Deallocate((void*)vertices_[i].x);
    Deallocate((void*)vertices_[i].y);
    Deallocate((void*)vertices_[i].z);

    Deallocate((void*)normals_[i].x);
    Deallocate((void*)normals_[i].y);
    Deallocate((void*)normals_[i].z);
  }

  Deallocate((void*)model_vertices_.x);
  Deallocate((void*)model_vertices_.y);
  Deallocate((void*)model_vertices_.z);

  Deallocate((void*)model_normals_.x);
  Deallocate((void*)model_normals_.y);
  Deallocate((void*)model_normals_.z);

  Deallocate((void*)volume_);

  Deallocate((void*)normal_map_);
}

int FaceModeling::Run(const Depth *depth,cv::Mat cvRGBImg_color,Color *normal_map,
                      Matrix4f *transform) {
  // Segment the user's head from the depth image.
  if (head_segmentation_.Run(kMinDepth, kMaxDepth, kMaxDifference,
                             kMinHeadWidth, kMinHeadHeight,
                             kMaxHeadWidth, kMaxHeadHeight,
                             fx_, fy_, width_, height_, depth, depth_)) {
    printf("Unable to segment user's head from depth image\n");
    return -1;
  }

  // Upload the segmented depth image from the CPU to the GPU.
  Upload(segmented_depth_, depth_, sizeof(Depth) * width_ * height_);

  // Filter the depth image.
  variance_filter_.Run(width_, height_, segmented_depth_, filtered_depth_);
  bilateral_filter_.Run(kRegistrationBilateralFilterSigmaD,
                        kRegistrationBilateralFilterSigmaR,
                        width_, height_, filtered_depth_, denoised_depth_);

  // Construct depth image pyramid.
  Copy(depth_pyramid_[0], denoised_depth_, sizeof(Depth) * width_ * height_);
  for (int i = 1 ; i < kPyramidLevels; i++) {
    downsample_.Run(kDownsampleFactor, kDownsampleMaxDifference,
                    (width_ >> ((i - 1) * kDownsampleFactor)),
                    (height_ >> ((i - 1) * kDownsampleFactor)),
                    (width_ >> (i * kDownsampleFactor)),
                    (height_ >> (i * kDownsampleFactor)),
                    depth_pyramid_[i - 1], depth_pyramid_[i]);
  }

  // Construct the point-cloud pyramid by
  // back-projecting the depth image pyramid.
  for (int i = 0; i < kPyramidLevels; i++) {
    back_projection_.Run((width_ >> (i * kDownsampleFactor)),
                         (height_ >> (i * kDownsampleFactor)),
                         (fx_ / (1 << (i * kDownsampleFactor))),
                         (fy_ / (1 << (i * kDownsampleFactor))),
                         (cx_ / (1 << (i * kDownsampleFactor))),
                         (cy_ / (1 << (i * kDownsampleFactor))),
                         depth_pyramid_[i], vertices_[i], normals_[i]);
  }

  // Compute the center of mass of the point-cloud.
  Vertex center = centroid_.Run(width_, height_, vertices_[0]);

  // Register the current frame to the previous frame.
  if (!initial_frame_) {
    Matrix4f previous_transformation = transformation_;

    // Create Transformation that aligns the Current Frame
    // with the Previous Frame based on the centroids
    Matrix4f frame_transformation;
    frame_transformation.setIdentity();

    frame_transformation(0, 3) = previous_center_.x - center.x;
    frame_transformation(1, 3) = previous_center_.y - center.y;
    frame_transformation(2, 3) = previous_center_.z - center.z;

    // Approximate the current frame's global transformation.
    transformation_ = transformation_ * frame_transformation;

    // Perform ICP.
    for (int i = kPyramidLevels - 1; i >= 0; i--) {
      if (icp_.Run(kICPIterations[i],
                   kMinCorrespondences[i + 1], kMinCorrespondences[i],
                   kDistanceThreshold[i + 1], kDistanceThreshold[i],
                   kNormalThreshold[i + 1], kNormalThreshold[i],
                   kMaxRotation, kMaxTranslation,
                   fx_, fy_, cx_, cy_,
                   (width_ >> (i * kDownsampleFactor)),
                   (height_ >> (i * kDownsampleFactor)),
                   width_, height_, vertices_[i], normals_[i],
                   model_vertices_, model_normals_,
                   previous_transformation, transformation_)) {
        printf("Unable to register depth image\n");
        transformation_ = previous_transformation;

        if (failed_frames_ >= kMaxFailedFrames) {
          transformation_.setIdentity();

          ray_casting_.Run(kMaxDistance, kMaxTruncation, kVolumeSize,
                           kVolumeDimension, kVoxelDimension, min_weight_,
                           width_, height_, fx_, fy_, cx_, cy_, volume_center_,
                           transformation_, volume_, model_vertices_,
                           model_normals_, normal_map_);

          Download(normal_map, normal_map_, sizeof(Color) * width_ * height_);
          previous_center_ = volume_center_;
        }

        failed_frames_++;
        return -1;
      }
    }
  }
  else {
    // Set the center of the volume to the
    // center of mass of the initial frame.
    volume_center_ = center;
  }

  failed_frames_ = 0;

  // Integrate the segmented depth image into the volumetric model.
  bilateral_filter_.Run(kIntegrationBilateralFilterSigmaD,
                        kIntegrationBilateralFilterSigmaR,
                        width_, height_, filtered_depth_, denoised_depth_);

  volumetric_.Run(kVolumeSize, kVolumeDimension, kVoxelDimension,
                  kMaxTruncation, kMaxWeight, width_,  height_,
                  fx_, fy_, cx_, cy_, volume_center_, transformation_.inverse(),
                  denoised_depth_, normals_[0], volume_);

  min_weight_ = MIN(min_weight_ + kMinWeightPerFrame / kMaxWeight,
                    kMaxMinWeight / kMaxWeight);

  // Render the volume using ray casting. Update the model point-cloud
  // for the next frame's registration step. Generate the normal map of
  // the model to display the current state of the model to the user.
  ray_casting_.Run(kMaxDistance, kMaxTruncation, kVolumeSize, kVolumeDimension,
                   kVoxelDimension, min_weight_, width_, height_,
                   fx_, fy_, cx_, cy_, volume_center_, transformation_,
                   volume_, model_vertices_, model_normals_, normal_map_);

  // Download the normal map from the GPU to the CPU.
  if (normal_map != NULL)
    Download(normal_map, normal_map_, sizeof(Color) * width_ * height_);
  if (transform != NULL)
    *transform = transformation_;

   
   //why---------------------------------2s download and save
   if(count==10)
   {
  
   

    

    Download(denoised_depth_c, denoised_depth_ ,sizeof(Depth)*width_*height_);
    Download(cloud_1.x,vertices_[0].x,sizeof(float) * width_ * height_);
    Download(cloud_1.y,vertices_[0].y,sizeof(float) * width_ * height_);
    Download(cloud_1.z,vertices_[0].z,sizeof(float) * width_ * height_);
   
  /*
    for(int i=0;i<width_ * height_;i++)
    {//show the data
      //printf("%d ",normal_map[i].r);
      //printf("%d ",normal_map[i].g);
      //printf("%d ",normal_map[i].b);
      printf("%f ",cloud_1.x[i]);
      printf("%f ",cloud_1.y[i]);
      printf("%f ",cloud_1.z[i]);
    }
    
   */
     

    //change format cloud s
   
    for(int i=0 ; i<height_;i++)
     for(int j=0; j<width_;j++)
     {
        int idx = i*height_+j;
        cloud_2.points[idx].x = cloud_1.x[idx];
        cloud_2.points[idx].y = cloud_1.y[idx];
        cloud_2.points[idx].z = cloud_1.z[idx];
        //printf("%f ",cloud_2.points[idx].x);
        //printf("%f ",cloud_2.points[idx].y);
       // printf("%f ",cloud_2.points[idx].z);
     }
    
  
    
    //max_pixel = std::max(denoised_depth_);
    //cv::Mat cvRawImg16U(height_,width_, CV_16UC1,denoised_depth_);
    //cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0/(cvRawImg16U.getMaxPixelValue()));
    //change format e
    
   
    //save
    //sprintf(pname,"%s%d%s","/home/gm/why/Depth-Image-Processing-master/build/applications/face_modeling/Images/Color_",count, ".jpg");
    /*
    
   */
    
    cv::Mat cvRGBImg( height_,width_, CV_8UC3,normal_map);
    //cv::cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
    sprintf(pname,"%s%d%s","Normal_",count_num, ".jpg");
    cv::imwrite(pname,cvRGBImg);


    
    sprintf(pname,"%s%d%s","Depth_",count_num, ".jpg");
    cv::Mat cvDepthImg(height_,width_,CV_16UC1,(void*)denoised_depth_c); 

    //cv::imshow(cvDepthImg);
    cv::imwrite(pname,cvDepthImg); 
    //cvDepthImg.release();
    
    sprintf(pname,"%s%d%s","Color_",count_num, ".jpg");
    cv::imwrite(pname,cvRGBImg_color);

 
    sprintf(pname,"%s%d%s","Cloud_",count_num, ".pcd");   
    pcl::io::savePCDFileASCII (pname, cloud_2);

    printf("SAVE SUCCEED %d",count);
    //save end
    count_num++;
    count = 0;
   }

  

  //update count
  count++;
  //why-------------------------------------------------2e

  // Update Model Center
  previous_center_ = center;

  initial_frame_ = false;
  return 0;
}

void FaceModeling::Model(Mesh *mesh) {
  // Allocate volume on the CPU.
  Voxel *volume = new Voxel[kVolumeSize * kVolumeSize * kVolumeSize];

  // Download the volume from the GPU to the CPU.
  Download((void*)volume, volume_, sizeof(Voxel) *
           kVolumeSize * kVolumeSize * kVolumeSize);

  // Construct mesh using marching cubes.
  marching_cubes_.Run(kVolumeSize, kVolumeDimension, kVoxelDimension,
                      min_weight_, volume_center_, volume, mesh);

  delete [] volume;
}

} // namespace dip
