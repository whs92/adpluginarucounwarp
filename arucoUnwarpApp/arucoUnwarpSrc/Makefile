TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for the source files that define template functions.
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
    CODE_CXXFLAGS=
endif

# Needed for new threading function
CODE_CXXFLAGS=-std=c++11

DBD += NDPluginArucoUnwarp.dbd

INC += NDPluginArucoUnwarp.h

LIBRARY_IOC += NDPluginArucoUnwarp

NDPluginArucoUnwarp_SRCS += NDPluginArucoUnwarp.cpp

#TODO: When compiling external opencv+zbar test, I needed to run:
# g++ test.cpp $(pkg-config --libs opencv --cflags) $(pkg-config --libs zbar --cflags) -o check
#Must link Opencv and zbar libraries here

ifdef OPENCV_INCLUDE
    USR_INCLUDES += -I$(OPENCV_INCLUDE)
endif
ifdef OPENCV_LIB
    NDPluginArucoUnwarp_DIR += $(OPENCV_LIB)
endif

include $(AREA_DETECTOR)/ADCore/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
