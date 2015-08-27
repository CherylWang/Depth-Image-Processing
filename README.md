Depth Image Processing
======================

Depth Image Processing (DIP) is a library of research code created by
[Greg Meyer](http://www.ifp.illinois.edu/~gmeyer3/) at the University of
Illinois at Urbana-Champaign. The following projects are currently implemented
in DIP:

* [Real-time 3D face modeling with a commodity depth camera]
(http://www.ifp.illinois.edu/~gmeyer3/papers/icme2013.pdf)
* [Improving Face Detection with Depth]
(http://www.ifp.illinois.edu/~gmeyer3/papers/icip2015.pdf)

DIP can be compiled and installed using CMake.

### Requirements ###

* CUDA 5.0
* Eigen 3.0.0
* HDF5
* OpenCV
* OpenNI2
* OpenGL/GLUT
* PCL 1.7

when run face_modeling
type in : 
 ./face_modeling filename h         --------read frames from HDF5 file
 ./face_omdeling filename 任意字符   --------read frames from .oni file 
