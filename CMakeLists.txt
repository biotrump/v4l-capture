cmake_minimum_required(VERSION 2.8)
PROJECT( demo)

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

