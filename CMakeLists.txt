cmake_minimum_required(VERSION 2.8)

#SET(OPENCV_CPP_SAMPLES_REQUIRED_DEPS opencv_core opencv_flann opencv_imgproc
#    opencv_highgui opencv_ml opencv_video opencv_objdetect opencv_photo opencv_nonfree
#    opencv_features2d opencv_calib3d opencv_legacy opencv_contrib opencv_stitching opencv_videostab)

PROJECT( demo)
#static linking, we have to set the package fining path to my own local opencv build.
#otherwise it will use shared lib in /usr/local/share/openCV
#my mozilla PC
#set(OpenCV_DIR "#set(OpenCV_DIR ${(OpenCV_BUILD})
set(OpenCV_DIR "/home/thomas/build/biotrump-cv/out/openCV/2.4.9.x-static")
#acer s3 ubuntu opencv static linking build
#set(OpenCV_DIR "/home/thomas/build/openCV/2.4.9-static")
FIND_PACKAGE( OpenCV REQUIRED)

#my mozilla PC
#set(V4L2_DIR "/home/thomas/build/biotrump-cv/v4l2-lib")
#FIND_PACKAGE( V4L2 REQUIRED)

#include_directories(${V4L2_DIR})

#openMP to avoid link error : omp_xxx error!
# undefined reference to `GOMP_loop_dynamic_start
find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

ADD_EXECUTABLE( demo
	demo.c
	)

#dynamic or static link
TARGET_LINK_LIBRARIES( demo ${OpenCV_LIBS} )

ADD_EXECUTABLE( demo1
	demo1.c
	)

#dynamic or static link
TARGET_LINK_LIBRARIES( demo1 ${OpenCV_LIBS} )

