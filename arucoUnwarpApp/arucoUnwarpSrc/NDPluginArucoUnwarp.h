/*
 * NDPluginArucoUnwarp.h
 *
 * Header file for EPICS ArucoUnwarp/QR reader plugin
 * Author: Jakub Wlodek
 * Co-author: Kazimier Gofron
 *
 * Created on: December 5, 2017
 * Last Updated: January 4, 2019
*/

#ifndef NDPluginArucoUnwarp_H
#define NDPluginArucoUnwarp_H

//two includes
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <thread>

using namespace std;
using namespace cv;


//include base plugin driver
#include "NDPluginDriver.h"

//version numbers
#define ArucoUnwarp_VERSION 1
#define ArucoUnwarp_REVISION 0
#define ArucoUnwarp_MODIFICATION 0

/* Here I will define all of the output data types once the database is written */

#define NDPluginArucoUnwarpShowMappingString "SHOW_MAPPING"          //asynInt32
#define NDPluginArucoUnwarpFindHomographyString "FIND_HOMOGRAPHY"          //asynInt32
#define NDPluginArucoUnwarpShowMarkersString "SHOW_MARKERS"          //asynInt32

/* class that does ArucoUnwarpcode readings */
class NDPluginArucoUnwarp : public NDPluginDriver {
   public:
    NDPluginArucoUnwarp(const char *portName, int queueSize, int blockingCallbacks,
                const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
                size_t maxMemory, int priority, int stackSize);

    //~NDPluginArucoUnwarp();

    void processCallbacks(NDArray *pArray);
    asynStatus ArucoUnwarpcode_image_callback(NDArray* pArray);
    //virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

   protected:
    //in this section i define the coords of database vals

  
    int NDPluginArucoUnwarpShowMapping;
#define ND_ArucoUnwarp_FIRST_PARAM NDPluginArucoUnwarpShowMapping
    int NDPluginArucoUnwarpShowMarkers;
    //boolean to decide to use existing homography or find a new one
    int NDPluginArucoUnwarpFindHomography;
#define ND_ArucoUnwarp_LAST_PARAM NDPluginArucoUnwarpFindHomography

   private:
    // processing thread
    bool processing = false;

    // image type conversion functions
    void printCVError(cv::Exception &e, const char *functionName);
    asynStatus ndArray2Mat(NDArray *pArray, NDArrayInfo *arrayInfo, Mat &img);
    asynStatus mat2NDArray(NDArray *pScratch, Mat &img);


    //function to generate aruco code
    asynStatus h_cat(Mat &img1, Mat &img2, Mat &out_img );
    asynStatus show_matches(Mat &img1, Mat &img2, Mat &out_img,vector<cv::Point2f> points1, vector<cv::Point2f> points2  );
 
    asynStatus find_charuco_arrays(Mat &img,  int dict, InputOutputArrayOfArrays markerCorners, InputOutputArray markerIds, InputOutputArray charucoCorners, InputOutputArray charucoIds);

    //Generate a scaled image of the charuco board
    asynStatus gen_charuco_img(Mat &img,  int dict );

};

#define NUM_ArucoUnwarp_PARAMS ((int)(&ND_ArucoUnwarp_LAST_PARAM - &ND_ArucoUnwarp_FIRST_PARAM + 1))

#endif
