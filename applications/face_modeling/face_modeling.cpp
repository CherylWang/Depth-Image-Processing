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

//---------------------------------------------why
#include<sys/stat.h>
#include<sys/types.h>
//---------------------------------------------why

// OpenGL
#include <GL/glut.h>


#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <GL/freeglut_ext.h>
using namespace cv;

// DIP
#include <dip/cameras/dumpfile.h>
#include <dip/cameras/onifile.h>
#include <dip/cameras/primesense.h>
#include <dip/cameras/softkinetic.h>
#include <dip/common/types.h>
#include <dip/io/objfile.h>
#include <dip/projects/facemodeling.h>
#include <dip/surface/mesh.h>

using namespace dip;

const int kWindowWidth = 640;
const int kWindowHeight = 480;
     
const int kFramesPerSecond = 60;

Camera *g_camera = NULL;
FaceModeling *g_modeling = NULL;
OBJFile *g_obj_file = NULL;
GLuint g_texture;


Depth *g_depth = NULL;
Color *g_normals = NULL;
Color *g_color = NULL;

//-----------------------------------------------why1 s
char   folder_name[100];
char   objFile[100];
bool   save_Depth   = false;   
bool   save_Color   = false;
bool   save_Normal  = false;
bool   save_Cloud   = false;
bool   pause_run        = false;
int    cur_frame = 0;
//-----------------------------------------------why1 e



//-----------------------------------------------why add s
void print_info()
{
   printf("---------------------------------------------\n");
   printf("press : ESC to save current and exit\n");
   printf("press : 1   to open/close save_depth\n");
   printf("press : 2   to open/close save_color\n");
   printf("press : 3   to open/close save_normal\n");
   printf("press : 4   to open/close save_cloud\n");
   printf("press : 5   to reset the work, whcih mean the current folder will be deleted\n");
   printf("press : 6   to save current obj and reset\n");
   printf("press : 7   to pause/continue\n");
}



void new_Folder()
{//new folder and objFile    
   printf("please type in the name of the current Folder  eg: why \n");
   scanf("%s",folder_name);
   mkdir(folder_name,S_IRWXU);
   sprintf(objFile,"%s%s",folder_name,"/1.obj"); 
   g_obj_file = new OBJFile(objFile, CREATE_OBJ);
}



void save_obj()
{
  if (g_obj_file->enabled()) {
    Mesh mesh;
    g_modeling->Model(&mesh);
    g_obj_file->Write(&mesh);
  }
}


void reset()
{// 1:jump out of glutMainLoop 2:reset flags 3:reset variabiles in facemodeling.cpp 4:restart glutMainLoop

    //glutLeaveMainLoop();
    printf("jump out already,  begin to reset......................................................\n");
    save_Depth   = false;   
    save_Color   = false;
    save_Normal  = false;
    save_Cloud   = false;
    pause_run    = false;
    printf("clear flags over......................................................\n");        
 //   g_modeling->reset_facemodeling();
    print_info();
    delete g_obj_file;
    g_obj_file = NULL;
    new_Folder();
  if (g_modeling != NULL)
    delete g_modeling;
 g_modeling = new FaceModeling(g_camera->width(DEPTH_SENSOR),
                                g_camera->height(DEPTH_SENSOR),
                                g_camera->fx(DEPTH_SENSOR),
                                g_camera->fy(DEPTH_SENSOR),
                                g_camera->width(DEPTH_SENSOR) / 2.0f,
                                g_camera->height(DEPTH_SENSOR) / 2.0f);

    //glutMainLoop();
}


void close() {

  save_obj();
  if (g_camera != NULL)
    delete g_camera;
  if (g_modeling != NULL)
    delete g_modeling;
  if (g_obj_file != NULL)
    delete g_obj_file;

  if (g_depth != NULL)
    delete [] g_depth;
  if (g_normals != NULL)
    delete [] g_normals;

  exit(0);
}


void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  // Quit Program
  case 27:
    close();      break;
  case 49:
    save_Depth = !save_Depth;   break;
  case 50:
    save_Color = !save_Color;   break;
  case 51:
    save_Normal = !save_Normal; break;
  case 52:
    save_Cloud  = !save_Cloud;  break;
  case 53:
    //delete abandoned folder  ---------uncompleted
    //???
    //delete abandoned folder  ---------uncompleted
    reset();       break;
  case 54:
    save_obj();
    reset();       break;
  case 55:
    pause_run = !pause_run;break;
  }
}


//-----------------------------------------------why add e


void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glOrtho(0.0f, 1.0f, 0.0f, 1.0f, -10.0f, 10.0f);

//---------------------------------------------------------------why pause s
  
if(!pause_run){

  // Update Frame
  if (g_camera->Update(g_depth)) {
    printf("Unable to Update depth Frame\n");
    close();
  }

   if(g_camera->Update(g_color))
    {
    printf("Unable to Update color Frame\n");
    close();
    }
   cv::Mat cvBGRImg;
   cv::Mat cvRGBImg_color(g_camera->height(COLOR_SENSOR),
                          g_camera->width(COLOR_SENSOR),
                          CV_8UC3,g_color);

   cv::cvtColor(cvRGBImg_color,cvBGRImg, CV_RGB2BGR);


  // Update Model
  g_modeling->Run(save_Depth, save_Color, save_Normal, save_Cloud , folder_name, g_depth,cvBGRImg,g_normals); //only change 

}//if(!pause_run) 
//-------------------------------------------------------------why pause e 
 

 // Update Texture
  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, g_texture);

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, g_camera->width(DEPTH_SENSOR),
                  g_camera->height(DEPTH_SENSOR), GL_RGB, GL_UNSIGNED_BYTE,
                  g_normals);

  glDisable(GL_TEXTURE_2D);

  // Display Frame
  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, g_texture);

  glBegin(GL_QUADS);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(1.0f, 1.0f, 0.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(1.0f, 0.0f, 0.0f);
  glEnd();

  glDisable(GL_TEXTURE_2D);

  glutSwapBuffers();
}


void reshape(int w, int h) {
  glViewport(0, 0, w, h);
}


void timer(int fps) {
  glutPostRedisplay();
  glutTimerFunc(1000 / fps, timer, fps);
}

int main(int argc, char **argv){
  if (argc > 3|| argc<1||argc ==2){
    printf("Usage: %s [filename  'h'/other_char ]\n", argv[0]);
    printf("do not need <Mesh File> here\n");
    return -1;
  }
  
  //-------------------------------------why s
  print_info();
  
  new_Folder();
  //-------------------------------------why e

  glutInit(&argc, argv);

  // Initialize Camera
 // Initialize Camera
if (argc > 2){
  
    if(argv[2] == "h")
    {
        g_camera = new DumpFile(argv[1]);
    } else{
        g_camera = new OniFile(argv[1]);
    }
} else{
#ifndef SOFTKINETIC
    g_camera = new PrimeSense();
#else
    g_camera = new SoftKinetic();
#endif
}

  if (!g_camera->enabled()) {
    printf("Unable to Open Camera\n");
    return -1;
  }

  // Initialize 3D face modeling.
  g_modeling = new FaceModeling(g_camera->width(DEPTH_SENSOR),
                                g_camera->height(DEPTH_SENSOR),
                                g_camera->fx(DEPTH_SENSOR),
                                g_camera->fy(DEPTH_SENSOR),
                                g_camera->width(DEPTH_SENSOR) / 2.0f,
                                g_camera->height(DEPTH_SENSOR) / 2.0f);


// Initialize Buffers
  g_depth = new Depth[g_camera->width(DEPTH_SENSOR) *
                      g_camera->height(DEPTH_SENSOR)];
  g_normals = new Color[g_camera->width(DEPTH_SENSOR) *
                        g_camera->height(DEPTH_SENSOR)];

  g_color = new Color[g_camera->height(COLOR_SENSOR)*g_camera->width(COLOR_SENSOR)];


  // Initialize OpenGL
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(kWindowWidth, kWindowHeight);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("3D Face Modeling");

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutTimerFunc(1000 / kFramesPerSecond, timer, kFramesPerSecond);

  // Initialize Texture
  glEnable(GL_TEXTURE_2D);

  glGenTextures(1, &g_texture);
  glBindTexture(GL_TEXTURE_2D, g_texture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_camera->width(DEPTH_SENSOR),
               g_camera->height(DEPTH_SENSOR), 0, GL_RGB, GL_UNSIGNED_BYTE,
               NULL);

  glDisable(GL_TEXTURE_2D);
  
 
  
  glutMainLoop();
       
  
  return 0;
}
