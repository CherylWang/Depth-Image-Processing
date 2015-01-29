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

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <chrono>

// OpenGL
#include <GL/glut.h>

// DIP
#include <dip/cameras/camera.h>
#include <dip/cameras/dumpfile.h>
#include <dip/cameras/primesense.h>
#include <dip/common/types.h>
#include <dip/io/hdf5wrapper.h>
#include <dip/segmentation/facemasker.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace dip;
using namespace std;
using namespace std::chrono;

const int kWindowWidth = 640;
const int kWindowHeight = 480;

const int kFramesPerSecond = 30;

const bool kDownsample = true;
const int kMinDepth = 256;
const int kMinPixels = 10;
const int kOpenSize = 2;
const int kHeadWidth = 150;
const int kHeadHeight = 150;
const int kHeadDepth = 100;
const int kFaceSize = 150;
const int kExtendedSize = 50;

const char kCascade[] = "haarcascade_frontalface_default.xml";

bool g_masking = false;

CascadeClassifier g_cascade;
FaceMasker *g_masker = NULL;

Camera *g_camera = NULL;
HDF5Wrapper *g_dump = NULL;

Depth *g_depth = NULL;
Depth *g_downsampled_depth = NULL;
Color *g_color = NULL;

int g_frame = 0;

GLuint g_texture;

void close() {
  if (g_dump != NULL)
    delete g_dump;

  if (g_camera != NULL)
    delete g_camera;

  if (g_depth != NULL)
    delete [] g_depth;
  if (g_color != NULL)
    delete [] g_color;

  if (g_downsampled_depth != NULL)
    delete [] g_downsampled_depth;

  exit(0);
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glOrtho(0.0f, 1.0f, 0.0f, 1.0f, -10.0f, 10.0f);

  // Update depth image.
  if (g_camera->Update(g_depth)) {
    close();
  }

  // Update color image.
  if (g_camera->Update(g_color)) {
    close();
  }

  if (kDownsample) {
    int i = 0;
    for (int y = 0; y < g_camera->height(DEPTH_SENSOR) / 2; y++) {
      for (int x = 0; x < g_camera->width(DEPTH_SENSOR) / 2; x++, i++) {
        int j = (x << 1) + (y << 1) * g_camera->width(DEPTH_SENSOR);
        g_downsampled_depth[i] = g_depth[j];
      }
    }
  }

  high_resolution_clock::time_point start = high_resolution_clock::now();

  // Detect faces in color image.
  Mat image(g_camera->height(COLOR_SENSOR), g_camera->width(COLOR_SENSOR),
            CV_8UC3, g_color);

  // Eliminate sub-images using depth image.
  if (g_masking) {
    Size window_size = g_cascade.getOriginalWindowSize();

    if (kDownsample) {
      g_masker->Run(kMinDepth, kMinPixels, kOpenSize, kHeadWidth, kHeadHeight,
                    kHeadDepth, kFaceSize, kExtendedSize, window_size.width,
                    g_camera->width(DEPTH_SENSOR) / 2,
                    g_camera->height(DEPTH_SENSOR) / 2,
                    (g_camera->fx(DEPTH_SENSOR) + g_camera->fy(DEPTH_SENSOR)) /
                    4.0f, g_downsampled_depth, g_color);
    } else {
      g_masker->Run(kMinDepth, kMinPixels, kOpenSize, kHeadWidth, kHeadHeight,
                    kHeadDepth, kFaceSize, kExtendedSize, window_size.width,
                    g_camera->width(DEPTH_SENSOR),
                    g_camera->height(DEPTH_SENSOR),
                    (g_camera->fx(DEPTH_SENSOR) + g_camera->fy(DEPTH_SENSOR)) /
                    2.0f, g_depth, g_color);
    }
  }

  vector<Rect> faces;
  g_cascade.detectMultiScale(image, faces);

  high_resolution_clock::time_point stop = high_resolution_clock::now();
  duration<float> time_span = duration_cast<duration<float>>(stop - start);
  float timing = time_span.count() * 1000.0f;

  // Update Texture
  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, g_texture);

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, g_camera->width(COLOR_SENSOR),
                  g_camera->height(COLOR_SENSOR), GL_RGB, GL_UNSIGNED_BYTE,
                  g_color);

  glDisable(GL_TEXTURE_2D);

  // Display Frame
  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, g_texture);

  glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, 0.0f, 0.0f);
  glEnd();

  glDisable(GL_TEXTURE_2D);
  // Draw face rectangles.
  for (unsigned int i = 0; i < faces.size(); i++) {
    float left = (float)faces[i].x /
                  g_camera->width(COLOR_SENSOR);
    float right = (float)(faces[i].x + faces[i].width) /
                  g_camera->width(COLOR_SENSOR);
    float top = 1.0f - ((float)faces[i].y /
                g_camera->height(COLOR_SENSOR));
    float bottom = 1.0f - ((float)(faces[i].y + faces[i].height) /
                   g_camera->height(COLOR_SENSOR));

    glBegin(GL_LINE_LOOP);
      glVertex3f(left, top, 0.0f);
      glVertex3f(right, top, 0.0f);
      glVertex3f(right, bottom, 0.0f);
      glVertex3f(left, bottom, 0.0f);
    glEnd();
  }

  if ((g_dump != NULL) && g_dump->enabled()) {
    Mat f = Mat(g_camera->height(COLOR_SENSOR), g_camera->width(COLOR_SENSOR), CV_8UC3, g_color);

    char group[64];
    sprintf(group, "/FRAME%04d", g_frame);

    hsize_t dimensions = 3;
    float position[3];
    int valid = 0;

    g_dump->Read("POSITION", group, position, &dimensions, 1,
                 H5T_NATIVE_FLOAT);

    g_dump->Read("VALID", group, &valid, H5T_NATIVE_INT);

    if (valid) {
      // Project position to pixel location.
      float u, v;
      u = (g_camera->fx(DEPTH_SENSOR) * position[0] / position[2]) +
          (g_camera->width(DEPTH_SENSOR) / 2);
      v = g_camera->height(DEPTH_SENSOR) -
          ((g_camera->fy(DEPTH_SENSOR) * position[1] / position[2]) +
           (g_camera->height(DEPTH_SENSOR) / 2));


      int true_positives = 0, false_positives = 0;
      if (faces.size() > 0) {
        for (unsigned int i = 0; i < faces.size(); i++) {
          float left = faces[i].x;
          float right = faces[i].x + faces[i].width;
          float top = faces[i].y;
          float bottom = faces[i].y + faces[i].height;

          if ((u > left) && (u < right) && (v > top) && (v < bottom)) {
            true_positives++;
            rectangle(f, Point(faces[i].x, faces[i].y),
                      Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                      Scalar(0, 255, 0), 2);
          } else {
            false_positives++;
            rectangle(f, Point(faces[i].x, faces[i].y),
                      Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                      Scalar(255, 0, 0), 2);
          }
        }
      }

      printf("%d: %d, %d, %f\n", g_frame, true_positives, false_positives,
             timing);

      glDisable(GL_TEXTURE_2D);
      glPointSize(5);
      glBegin(GL_POINTS);
        glVertex2f(u / g_camera->width(DEPTH_SENSOR),
                   1.0f - (v / g_camera->height(DEPTH_SENSOR)));
      glEnd();
    }

    cvtColor(f, f, CV_RGB2BGR);
    static int frames = 0;
    char name[256];
    sprintf(name, "frame%04d-faces.png", frames);
    imwrite(name, f);
    frames++;
  }

  glutSwapBuffers();
  g_frame++;
}

void reshape(int w, int h) {
  glViewport(0, 0, w, h);
}

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  // Quit Program
  case 27:
    close();
    break;
  }
}

void timer(int fps) {
  glutPostRedisplay();
  glutTimerFunc(1000 / fps, timer, fps);
}

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    printf("Usage: %s <Masking> [Dump File]\n", argv[0]);
    return -1;
  }

  glutInit(&argc, argv);

  g_masking = atoi(argv[1]) ? true : false;

  // Initialize camera.
  if (argc < 3) {
    g_camera = new PrimeSense();
  } else {
    g_camera = new DumpFile(argv[2]);
    g_dump = new HDF5Wrapper(argv[2], READ_HDF5);
  }

  if (!g_camera->enabled()) {
    printf("Unable to Open Camera\n");
    return -1;
  }

  // Initialize buffers.
  g_depth = new Depth[g_camera->width(DEPTH_SENSOR) *
                      g_camera->height(DEPTH_SENSOR)];
  g_color = new Color[g_camera->width(COLOR_SENSOR) *
                      g_camera->height(COLOR_SENSOR)];

  if (kDownsample) {
    g_downsampled_depth = new Depth[g_camera->width(DEPTH_SENSOR) *
                                    g_camera->height(DEPTH_SENSOR) / 4];
  }

  // Initialize face classifier.
  if (!g_cascade.load(kCascade)) {
    printf("Failed to load cascade classifier.\n");
    return -1;
  }

  // Initialize face masker.
  if (g_masking) {
    g_masker = new FaceMasker;
    Ptr<CascadeClassifier::MaskGenerator> masker_ptr(g_masker);
    g_cascade.setMaskGenerator(masker_ptr);
  }

  // Initialize OpenGL.
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(kWindowWidth, kWindowHeight);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Face Detection");

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutTimerFunc(1000 / kFramesPerSecond, timer, kFramesPerSecond);

  // Initialize texture.
  glEnable(GL_TEXTURE_2D);

  glGenTextures(1, &g_texture);
  glBindTexture(GL_TEXTURE_2D, g_texture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_camera->width(COLOR_SENSOR),
               g_camera->height(COLOR_SENSOR), 0, GL_RGB,
               GL_UNSIGNED_BYTE, NULL);

  glDisable(GL_TEXTURE_2D);

  glutMainLoop();

  return 0;
}
