# ADPluginArucoUnwarp

A plugin for EPICS Area Detector using OpenCV libraries which detects ChArUco codes in an image 
and then uses the points from those detected codes to find a homography transform between the
input image and a generated image of a known size. The transform is then applied

Primary Author:    	Will Smith, 

### Installation and dependancies

The ADPluginArucoUnwarp plugin works with the EPICS control system and thus requires a supported version of
EPICS base as well as synApps. synApps will include area detector and asyn, both required for ADPluginArucoUnwarp.

Two external libraries are required as well: OpenCV, which is used for image manipulation.Opencv can be built from source by cloning the
respective repositories from github, or if on a debin/ubuntu system, slightly older versions can
be downloaded using the package manager with the following commands:

```
sudo apt install libopencv-dev 
sudo apt install libopencv-contrib-dev
```

In addition, the numbered versions of these packages may be needed. For example, on the ubuntu 20.04 machine
this was tested on, libopencv4.2 was used.

It is also possible to build both opencv from source

Once EPICS base, synApps and OpenCV are all installed, some changes need to be made to the
configuration files within area detector. First, enter into your areaDetector directory, and
clone the ADPluginArucoUnwarp repository. It should be on the same level as ADCore.

Next, enter the ADCore directory, enter ADApp, and open the commonDriverMakefile file,
 and ensure that the following is added and uncommented after ADPluginArucoUnwarp:

```
ifdef ADPLUGINARUCOUNWARP
  $(DBD_NAME)_DBD += NDPluginArucoUnwarp.dbd
  PROD_LIBS	  += NDPluginArucoUnwarp
  ifdef OPENCV_LIB
    opencv_core_DIR +=$(OPENCV_LIB)
    PROD_LIBS       += opencv_core opencv_imgproc opencv_highgui opencv_aruco
  else
    PROD_SYS_LIBS   += opencv_core opencv_imgproc opencv_highgui opencv_aruco
  endif
endif
```

This will link the necessary OpenCV libraries when making area detector.

Return to the ADCore directory, and now enter the iocBoot directory. Open commonPlugins.cmd,
and ensure the following is uncommented:

```
# Optional: load NDPluginArucoUnwarp plugin
NDArucoUnwarpConfigure("ARUCO1", $(QSIZE), 0, "$(PORT)", 0, 0, 0, 0, 0, $(MAX_THREADS=5))
dbLoadRecords("$(ADPLUGINARUCOUNWARP)/db/NDPluginArucoUnwarp.template",  "P=$(PREFIX),R=Aruco1:, PORT=ARUCO1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
set_requestfile_path("$(ADPLUGINARUCOUNWARP)/arucoUnwarpApp/Db")

```

This will add ADPluginBar to the boot operation when the ioc is run.  

Next in the configure directory at the top level of areaDetector, open the RELEASE_PRODS.local file, and add the following:

```
# Load the ADPluginArucoUnwarp plugin
ADPLUGINARUCOUNWARP=$(AREA_DETECTOR)/ADPluginArucoUnwarp

```

This will tell areaDetector to include ADPluginBar at compilation.   

Then, in the CONFIG_SITE.local.$(YOUR HOST) file, ensure that the following is defined:  

```
# OPENCV_LIB and OPENCV_INCLUDE variables should not be defined if using the opencv system library in a default location
WITH_OPENCV     = YES 
OPENCV          = /usr
#OPENCV_LIB     = $(OPENCV)/lib64
#OPENCV_INCLUDE = -I$(OPENCV)/include

```

Simply replace the variable to give your path to OpenCV includes. Make sure that WITH_OPENCV is set to "YES"

Next, in CONFIG_SITE.local, in the same directory, make sure that WITH_OPENCV and OPENCV_EXTERNAL are both set to "YES"

Once you have done all of this, compile ADSupport, then ADCore, and then run

```
make -sj
```

in the ADPluginArucoUnwarp directory to compile it.

You have now installed the ADPluginArucoUnwarp Plugin.

### Usage

To use ADPluginBar with CSS, place the provided .bob screens into your CSS setup, and link to it
appropriately. The plugin supports 8 and 16 bit images in Mono or RGB formats. In order to view the unwarped image live, you may use any EPICS image viewer such as ImageJ, NDPluginStdArrays, or NDPluginPva, by setting the NDArrayPort to ARUCO1, or whichever port the plugin was assigned. This will display the image that the plugin processes.

Intended usage is that a reference screen is created and printed onto a flat surface. The parameters for that reference screen are added to this plugin. The plugin is then fed an image containing the screen or some part of it. ChArUco codes are found and then those points are used to unwarp the input image and provide some scaling. 
### Process Variables Supported

PV		|  Comment
----------------|---------------
RefBoardSize | The size in mm of the reference board
MmPx | The number of millimeters per pixel in the reference board 
RefSquareSize | The number of pixels in each square in the ChArUco board
RefMarkerSize | The number of pixels in each marker in the ChArUco board
RefBoardSquares | The number of squares in the ChArUco board
ArUcoDict | The dictionary used to generate the ArUco Board (Current disconnected)
Scaling | The scaling applied to the image during the unwarping
ShowMapping |  When set to 1 the mapping between all detected points in the input and the reference image will be shown. Useful for debug
FindHomography | When set to 1 a new homography will be determined for every new input image. When set to 0 it will use the last determined homography matrix
ShowMarkers | When Set to 1 will show the ChArUco markers on the output image
IncludeArUco | When set to 1 both ArUco Marker Corners and ChArUco Corners will be used in the homography. Otherwise, only ChArUco. The Latter is useful because they are more stable.
HomographyAvailable_RBV | When this PV is asserted a previous homography is available to perform the unwarp 



### Known limitations

There are some limitiations to the current release of the NDPluginBar plugin:

* The acquisition of ArUco Marker Corners is not as stable as the ChArUco Corners
* A lot of the parameters used in the steps to find the aruco codes are hard coded and still need to be exposed as EPICS PV'same
* 

For any other issues or limitations, please feel free to submit an issue on the ADPluginArucoUnwarp github page
