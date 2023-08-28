/*
 * NDPluginArucoUnwarp.cpp
 *
 * Plugin for EPICS area detector which searches for ChArUco codes and then uses
 * the points found to unwarp an image and apply scaling
 * Extends from the base NDPlugin Driver and overrides its processCallbacks function
 * The OpenCV computer vision library is used
 *
 * Author: Will Smith
 *
 * Created on: August 28, 2023
 * 
 *
*/

//include some standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <string>
#include <sstream>

//include epics/area detector libraries
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <iocsh.h>
#include "NDArray.h"
#include "NDPluginArucoUnwarp.h"

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

//some basic namespaces
using namespace std;
using namespace cv;

static const char *driverName = "NDPluginArucoUnwarp";
RNG rng(12345);

//------------------------------------------------------
// Utility functions for printing errors and for clearing previous codes
//------------------------------------------------------

/**
 * Function for printing out OpenCV exception information
 */
void NDPluginArucoUnwarp::printCVError(cv::Exception &e, const char *functionName) {
    cout << "OpenCV Error in function " << functionName << " with code: " << e.code << ", " << e.err;
}


//------------------------------------------------------
// Image type conversion functions
//------------------------------------------------------

/**
 * Function that converts an NDArray into a Mat object.
 * Currently supports NDUInt8, NDInt8, NDUInt16, NDInt16 data types, and either mono or RGB 
 * Image types
 * 
 * If the image is in RGB, it is converted to grayscale before it is passed to the plugin, because
 * zArucoUnwarp requires a grayscale image for detection
 * 
 * @params[in]: pArray	-> pointer to an NDArray
 * @params[in]: arrayInfo -> pointer to info about NDArray
 * @params[out]: img	-> smart pointer to output Mat
 * @return: success if able to convert, error otherwise
 */
asynStatus NDPluginArucoUnwarp::ndArray2Mat(NDArray *pArray, NDArrayInfo *arrayInfo, Mat &img) {
    const char *functionName = "ndArray2Mat";
    // data type and num dimensions used during conversion
    NDDataType_t dataType = pArray->dataType;
    int ndims = pArray->ndims;
    // convert based on color depth and data type
    switch (dataType) {
        case NDUInt8:
            if (ndims == 2)
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_8UC1, pArray->pData);
            else
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_8UC3, pArray->pData);
            break;
        case NDInt8:
            if (ndims == 2)
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_8SC1, pArray->pData);
            else
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_8SC3, pArray->pData);
            break;
        case NDUInt16:
            if (ndims == 2)
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_16UC1, pArray->pData);
            else
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_16UC3, pArray->pData);
            break;
        case NDInt16:
            if (ndims == 2)
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_16SC1, pArray->pData);
            else
                img = Mat(arrayInfo->ySize, arrayInfo->xSize, CV_16SC3, pArray->pData);
            break;
        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error: unsupported data format, only 8 and 16 bit images allowed\n", driverName, functionName);
            return asynError;
    }

   
    return asynSuccess;
}

/**
 * Function that converts Mat back into NDArray. This function is guaranteed to 
 * have either a 8 bit or 16 bit color image, because the bounding boxes drawn
 * around the detected ArucoUnwarpcodes are blue. The rest of the image will appear
 * black and white, but the actual color mode will be RGB
 * 
 * @params[out]: pScratch -> output NDArray
 * @params[in]: img	-> Mat with ArucoUnwarpcode bounding boxes drawn
 * @return: success if converted correctly, error otherwise 
 */
asynStatus NDPluginArucoUnwarp::mat2NDArray(NDArray *pScratch, Mat &img) {
    const char *functionName = "mat2NDArray";
    int ndims = 3;
    Size matSize = img.size();
    NDDataType_t dataType;
    NDColorMode_t colorMode = NDColorModeRGB1;
    size_t dims[ndims];
    dims[0] = 3;
    dims[1] = matSize.width;
    dims[2] = matSize.height;

    if (img.depth() == CV_8U)
        dataType = NDUInt8;
    else if (img.depth() == CV_8S)
        dataType = NDInt8;
    else if (img.depth() == CV_16U)
        dataType = NDUInt16;
    else if (img.depth() == CV_16S)
        dataType = NDInt16;

    pScratch = pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);
    if (pScratch == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error, unable to allocate array\n", driverName, functionName);
        img.release();
        return asynError;
    }

    NDArrayInfo arrayInfo;
    pScratch->getInfo(&arrayInfo);

    size_t dataSize = img.step[0] * img.rows;

    if (dataSize != arrayInfo.totalBytes) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error, invalid array size\n", driverName, functionName);
        img.release();
        return asynError;
    }

    memcpy((unsigned char *)pScratch->pData, (unsigned char *)img.data, dataSize);
    pScratch->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);
    pScratch->pAttributeList->add("DataType", "Data Type", NDAttrInt32, &dataType);
    getAttributes(pScratch->pAttributeList);
    doCallbacksGenericPointer(pScratch, NDArrayData, 0);
    //release the array
    pScratch->release();
    // now that data is copied to NDArray, we don't need mat anymore
    img.release();
    return asynSuccess;
}




/*Function that uses opencv to locate ArUco and ChArUco points in a given board pattern
 * @params[in]: im -> the opencv image generated by converting the NDArray
 * @params[in]: int -> the aruco dict enum
 * @return: void
*/
asynStatus NDPluginArucoUnwarp::find_charuco_arrays(Mat &img,  int dict, InputOutputArrayOfArrays markerCorners, InputOutputArray markerIds, InputOutputArray charucoCorners, InputOutputArray charucoIds) {

    // init some temporary variables
    
    std::vector<std::vector<cv::Point2f>>  rejectedCandidates;
    
    const char* functionName = "find_charuco_arrays";
    
    // Create the reference board
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict);
    board = cv::aruco::CharucoBoard::create(5,5,20,14,dictionary);

    //initialise the detector parameters. Later we will read these from PV's
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    
    parameters = cv::aruco::DetectorParameters::create();

    //We previously inverted our board, so we need to make sure we can detect that as well
    parameters->detectInvertedMarker=true;

    //We are dealing with quite low light images, so it will help us to increase the range of thresholds we attempt
    parameters->adaptiveThreshWinSizeMax = 43;

    //We would like to use the points from our detection later for homography so we will take the extra processing for more precision
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    //We can define how precise we want to be'
    parameters->cornerRefinementMinAccuracy = 0.0001;
    
    //Find the arucoMarkers
    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s detected %d markers\n", driverName, functionName, markerIds.getMat().total());
    //Find the ChArUco Corners
    if(markerIds.getMat().total()>0){
        //Refine the selection using the stuff we know about the board
        cv::aruco::refineDetectedMarkers(img, board, markerCorners, markerIds,rejectedCandidates);

    }
    int drawPoints;
    getIntegerParam(NDPluginArucoUnwarpShowMarkers, &drawPoints);
    //Find the ChArUco Corners
    if(markerIds.getMat().total()>0){

        //Draw the markers on the image
        if(drawPoints==1){
            cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
        }
        

        cv::aruco::interpolateCornersCharuco(markerCorners,markerIds,img,board,charucoCorners, charucoIds);

        if(charucoIds.getMat().total()>0){
            //draw those as well
            if(drawPoints==1){
                cv::aruco::drawDetectedCornersCharuco(img, charucoCorners, charucoIds);
            }

        }
        

        
    }
    
    
    

    return asynSuccess;

}

/*
function which given two vectors of arrays will output the indices of the intersection of those two
*/

    


/*Function which given two sets of id's and points, will find the intersection between the two

*/


/*Function that uses opencv to generate a reference image of a given charucoBoard
 * @params[in]: im -> the opencv image generated by converting the NDArray
 * @params[in]: int -> the aruco dict enum
 * @return: void
*/
asynStatus NDPluginArucoUnwarp::gen_charuco_img(Mat &img,  int dict ){

    //Draw the board
    Mat temp_img;
    double radius_um = 25000/2;
    double um_pix = 25;
    double radius = int(ceil(radius_um/um_pix));
    int bsize = int(radius*1.3);
    double board_size_mm = bsize*um_pix/1000;
    const char* functionName = "gen_charuco_img";


    cv::Ptr<cv::aruco::Dictionary> dictionary;
    //Generate the dictionary
    dictionary = cv::aruco::getPredefinedDictionary(dict);

    //Generate the CharucoBoard
    cv::Ptr<cv::aruco::CharucoBoard> board;
    board = cv::aruco::CharucoBoard::create(5,5,20,14,dictionary);

    //Draw the board
    board->draw(cv::Size(bsize,bsize),temp_img); //we have to use -> because board is a pointer. 
    cvtColor(temp_img, temp_img, COLOR_GRAY2RGB);

    //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Info, The board is %d x %d pixels or %f x %f mm\n", driverName, functionName, bsize,bsize, board_size_mm,board_size_mm);
    
    //Invert the board and pass to the input image
    cv::bitwise_not(temp_img, img);

    return asynSuccess;

}

/* Function which will concatenate two images of different sizes
*/
asynStatus NDPluginArucoUnwarp::h_cat(Mat &img1, Mat &img2, Mat &out_img ){

 

    //Find Max height and width of images
    int max_height = max(img1.size().height,img2.size().height );
    int max_width = max(img1.size().width,img2.size().width );
    Scalar value( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Info, the max height is %d and the max width is %d\n", driverName, "h_cat",max_height,max_width);
    
    cv::copyMakeBorder(img1,img1,0,max_height-img1.size().height,0,max_width-img1.size().width,cv::BORDER_CONSTANT,value);
    cv::copyMakeBorder(img2,img2,0,max_height-img2.size().height,0,max_width-img2.size().width,cv::BORDER_CONSTANT,value);

    cv::hconcat(img1,img2,out_img);
    
   return asynSuccess;
}

/* Function which given two images and two sets of points in each image, 
   will plot lines from the points in one image to another image. 
   The images will be stacked
*/
asynStatus NDPluginArucoUnwarp::show_matches(Mat &img1, Mat &img2, Mat &out_img,vector<cv::Point2f> points1, vector<cv::Point2f> points2  ){
    
    // Form the concatenated image
    h_cat(img1, img2, out_img);

    for(unsigned int i; i<points1.size();i++){

        cv::Point2f pt1= points1[i];
        cv::Point2f pt2= points2[i];
        pt2.x = pt2.x+img1.size().width;
        Scalar value( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
        cv::line(out_img, pt1,pt2, value, 2);


    }
    
    return asynSuccess;
}

    



static void ArucoUnwarpcode_image_callback_wrapper(void* pPtr, NDArray* pArray){
    NDPluginArucoUnwarp* pPlugin = (NDPluginArucoUnwarp*) pPtr;
    pPlugin->ArucoUnwarpcode_image_callback(pArray);
}

asynStatus NDPluginArucoUnwarp::ArucoUnwarpcode_image_callback(NDArray* pArray){
    //start with an empty array for copy and array info
    NDArray *pScratch = NULL;
    Mat img, ref_img;
    const char* functionName = "ArucoUnwarpcode_image_callback";
    NDArrayInfo arrayInfo;
    // convert to Mat
    pArray->getInfo(&arrayInfo);

    asynStatus status;

    // Convert the array into the image matrix
    status = ndArray2Mat(pArray, &arrayInfo, img);
    if (status == asynError) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error converting to Mat\n", driverName, functionName);
        this->processing = false;
        return status;
    }
    
    // Generate the reference image
    status = gen_charuco_img(ref_img, cv::aruco::DICT_4X4_250);
    if (status == asynError) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error generating reference image\n", driverName, functionName);
        this->processing = false;
        return status;
    }
    
       

    // Find the charuco corners in the image and add markers
    // init some temporary variables
    std::vector<int> inpMarkerIds, inpCharucoIds,refMarkerIds, refCharucoIds;
    std::vector<std::vector<cv::Point2f>> inpMarkerCorners, refMarkerCorners, subRefMarkerCorners;
    std::vector<cv::Point2f> inpCharucoCorners, refCharucoCorners,subRefCharucoCorners,inpMarkerCornersExpanded, subRefMarkerCornersExpanded;

    status = find_charuco_arrays(img, cv::aruco::DICT_4X4_250, inpMarkerCorners,inpMarkerIds,  inpCharucoCorners,inpCharucoIds);
    if (status == asynError) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error finding charuco in input image\n", driverName, functionName);
        this->processing = false;
        return status;
    }
    
    status = find_charuco_arrays(ref_img, cv::aruco::DICT_4X4_250, refMarkerCorners,refMarkerIds,  refCharucoCorners,refCharucoIds);
    if (status == asynError) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error finding charuco in reference image\n", driverName, functionName);
        this->processing = false;
        return status;
    }

    
    //Itterate through the markers found in the input image. For each id, use the id as the index to the refMarkers to make a new
    // list that is a subset of the original refMarkerCorners
    
    unsigned int element;
    
    // generate the subset of reference markers which also apear in the input
    for(unsigned int index=0; index<inpMarkerCorners.size();index++){   
        
        // get the marker id from the vector of marker id's found in the input. Note it's reversed
        element = refMarkerIds.size() - inpMarkerIds[index]-1;

        // since the marker id is also it's index in the reference vector, we can use it as the index to retrieve the value
        subRefMarkerCorners.insert(subRefMarkerCorners.begin() + index, refMarkerCorners[element]);
    }

    // expand the inputMarkerCorners and subRefMarkerCorners from vectors of vectors of points to vectors of points

    
    
    for(unsigned int i=0; i<inpMarkerCorners.size();i++){
        //for each point in the corner set
        for(unsigned int j=0; j<4;j++){
            inpMarkerCornersExpanded.push_back(inpMarkerCorners[i][j]);
            subRefMarkerCornersExpanded.push_back(subRefMarkerCorners[i][j]);
        }
    }
    
    /// ----------- Now the same for the Charuco Corners ----------------
    
    for(unsigned int index=0; index<inpCharucoCorners.size();index++){
        // get the marker id from the vector of marker id's found in the input
        element = inpCharucoIds[index];

        // since the marker id is also it's index in the reference vector, we can use it as the index to retrieve the value
        subRefCharucoCorners.insert(subRefCharucoCorners.begin() + index, refCharucoCorners[element]);
    } 

    // ------------ Join vectors and determine homography -----------------

    subRefCharucoCorners.insert( subRefCharucoCorners.end(), subRefMarkerCornersExpanded.begin(), subRefMarkerCornersExpanded.end() );
    inpCharucoCorners.insert( inpCharucoCorners.end(), inpMarkerCornersExpanded.begin(), inpMarkerCornersExpanded.end() );

    
    
    int showMapping, findHomography;
    getIntegerParam(NDPluginArucoUnwarpShowMapping, &showMapping);
    getIntegerParam(NDPluginArucoUnwarpFindHomography, &findHomography);
    if(showMapping == 1){
        
        Mat cat_img;
        show_matches(img, ref_img, cat_img,inpCharucoCorners,subRefCharucoCorners);
        img = cat_img;

    }
    else if(findHomography ==1){

        // find the homography
    
        if(inpCharucoCorners.size()>3 && subRefCharucoCorners.size()>3){
            Mat H = cv::findHomography( inpCharucoCorners,subRefCharucoCorners, cv::RANSAC,2);
        
            //perform the warping
            Mat warped_img;
            cv::warpPerspective(img,warped_img, H, ref_img.size(),cv::INTER_LINEAR);
            img = warped_img;


        }

    }
    
    
    status = mat2NDArray(pScratch, img);
    if (status == asynError) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error, image not processed correctly\n", driverName, functionName);
    }
    this->processing = false;
    return status;
}

/**
 * Override of NDPluginDriver function. Used when selecting between ArucoUnwarpcodes
 * for which corners should be shown.
 * 
 * @params[in]: pasynUser	-> pointer to asyn User that initiated the transaction
 * @params[in]: value		-> value PV was set to
 * @return: success if PV was updated correctly, otherwise error
 */
/*
asynStatus NDPluginArucoUnwarp::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    status = setIntegerParam(function, value);
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s function = %d value=%d\n", driverName, functionName, function, value);

    if (function == NDPluginArucoUnwarpFindHomography) {
        
        setIntegerParam(, 0);
                setIntegerParam(NDPluginBarFindHomography, 0);
            }
        } else {
            for (i = 0; i < 4; i++) {
                setIntegerParam(cornerXPVs[i], codes_in_image[value].position[i].x);
                setIntegerParam(cornerYPVs[i], codes_in_image[value].position[i].y);
            }
        }
    } else if (function < ND_ArucoUnwarp_FIRST_PARAM) {
        status = NDPluginDriver::writeInt32(pasynUser, value);
    }
    callParamCallbacks();
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error writing Int32 val to PV\n", driverName, functionName);
    }
    return status;
}
*/


/* Process callbacks function inherited from NDPluginDriver.
 * Here it is overridden, and the following steps are taken:
 * 1) Check if the NDArray is mono, as zArucoUnwarp only accepts mono/grayscale images
 * 2) Convert the NDArray recieved into an OpenCV Mat object
 * 3) Decode ArucoUnwarpcode method is called
 * 4) (Currently Disabled) Show ArucoUnwarpcode method is called
 *
 * @params[in]: pArray -> NDArray recieved by the plugin from the camera
 * @return: void
*/
void NDPluginArucoUnwarp::processCallbacks(NDArray *pArray) {
    static const char *functionName = "processCallbacks";

    //call base class and get information about frame
    NDPluginDriver::beginProcessCallbacks(pArray);

    //unlock the mutex for the processing portion
    this->unlock();

    if(!this->processing){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Starting processing thread\n", driverName, functionName);
        this->processing = true;
        thread processing_thread(ArucoUnwarpcode_image_callback_wrapper, this, pArray);
        processing_thread.detach();
    }

    this->lock();

    callParamCallbacks();
}

//constructror from base class
NDPluginArucoUnwarp::NDPluginArucoUnwarp(const char *portName, int queueSize, int blockingCallbacks,
                         const char *NDArrayPort, int NDArrayAddr,
                         int maxBuffers, size_t maxMemory,
                         int priority, int stackSize)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks,
                     NDArrayPort, NDArrayAddr, 1, maxBuffers, maxMemory,
                     asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                     asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                     ASYN_MULTIDEVICE, 1, priority, stackSize, 1) {
    char versionString[25];

   

    //common params
    createParam(NDPluginArucoUnwarpShowMappingString, asynParamInt32, &NDPluginArucoUnwarpShowMapping);
    createParam(NDPluginArucoUnwarpShowMarkersString, asynParamInt32, &NDPluginArucoUnwarpShowMarkers);
    createParam(NDPluginArucoUnwarpFindHomographyString, asynParamInt32, &NDPluginArucoUnwarpFindHomography);
    
    setStringParam(NDPluginDriverPluginType, "NDPluginArucoUnwarp");
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", ArucoUnwarp_VERSION, ArucoUnwarp_REVISION, ArucoUnwarp_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);
    connectToArrayPort();
}

/**
 * External configure function. This will be called from the IOC shell of the
 * detector the plugin is attached to, and will create an instance of the plugin and start it
 * 
 * @params[in]	-> all passed to constructor
 */
extern "C" int NDArucoUnwarpConfigure(const char *portName, int queueSize, int blockingCallbacks,
                              const char *NDArrayPort, int NDArrayAddr,
                              int maxBuffers, size_t maxMemory,
                              int priority, int stackSize) {
    NDPluginArucoUnwarp *pPlugin = new NDPluginArucoUnwarp(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
                                           maxBuffers, maxMemory, priority, stackSize);
    return pPlugin->start();
}

/* IOC shell arguments passed to the plugin configure function */
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"frame queue size", iocshArgInt};
static const iocshArg initArg2 = {"blocking callbacks", iocshArgInt};
static const iocshArg initArg3 = {"NDArrayPort", iocshArgString};
static const iocshArg initArg4 = {"NDArrayAddr", iocshArgInt};
static const iocshArg initArg5 = {"maxBuffers", iocshArgInt};
static const iocshArg initArg6 = {"maxMemory", iocshArgInt};
static const iocshArg initArg7 = {"priority", iocshArgInt};
static const iocshArg initArg8 = {"stackSize", iocshArgInt};
static const iocshArg *const initArgs[] = {&initArg0,
                                           &initArg1,
                                           &initArg2,
                                           &initArg3,
                                           &initArg4,
                                           &initArg5,
                                           &initArg6,
                                           &initArg7,
                                           &initArg8};

/* Definition of the configure function for NDPluginArucoUnwarp in the IOC shell */
static const iocshFuncDef initFuncDef = {"NDArucoUnwarpConfigure", 9, initArgs};

/* link the configure function with the passed args, and call it from the IOC shell */
static void initCallFunc(const iocshArgBuf *args) {
    NDArucoUnwarpConfigure(args[0].sval, args[1].ival, args[2].ival,
                   args[3].sval, args[4].ival, args[5].ival,
                   args[6].ival, args[7].ival, args[8].ival);
}

/* function to register the configure function in the IOC shell */
extern "C" void NDArucoUnwarpRegister(void) {
    iocshRegister(&initFuncDef, initCallFunc);
}

/* Exports plugin registration */
extern "C" {
    epicsExportRegistrar(NDArucoUnwarpRegister);
}
