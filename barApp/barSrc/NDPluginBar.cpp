/*
 * NDPluginBar.cpp
 *
 * Barcode/QRcode reader plugin for EPICS area detector
 * Author: Jakub Wlodek
 *
 * Created December 3, 2017
 *
*/


#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <epicsMutex.h>
#include <epicsString.h>
#include <iocsh.h>
#include "NDArray.h"
#include "NDPluginBar.h"
#include <epicsExport.h>

#include <opencv2/opencv.hpp>
#include <zbar.h>

using namespace std;
using namespace cv;
using namespace zbar;
static const char *driverName="NDPluginBar";


/** Color Mode to CV Matrix
	NDInt8,     Signed 8-bit integer
	NDUInt8,    Unsigned 8-bit integer
	NDInt16,    Signed 16-bit integer
	NDUInt16,   Unsigned 16-bit integer
	NDInt32,    Signed 32-bit integer
	NDUInt32,   Unsigned 32-bit integer
	NDFloat32,  32-bit float
	NDFloat64   64-bit float
	NDColorModeMono,    Monochromatic image
	NDColorModeBayer,   Bayer pattern image, 1 value per pixel but with color filter on detector
	NDColorModeRGB1,    RGB image with pixel color interleave, data array is [3, NX, NY]
	NDColorModeRGB2,    RGB image with row color interleave, data array is [NX, 3, NY]
	NDColorModeRGB3,    RGB image with plane color interleave, data array is [NX, NY, 3]
	NDColorModeYUV444,  YUV image, 3 bytes encodes 1 RGB pixel
	NDColorModeYUV422,  YUV image, 4 bytes encodes 2 RGB pixel
	NDColorModeYUV411   YUV image, 6 bytes encodes 4 RGB pixels
	NDArray         OpenCV
	=========       ==========
	NDInt8          CV_8S
	NDUInt8         CV_8U
	NDInt16         CV_16S
	NDUInt16        CV_16U
	NDInt32         CV_32S
	NDUInt32        CV_32U
	NDFloat32       CV_32F
	NDFloat64       CV_64F
	ND_BayerPatern  OpenCV
	==============  ======
	NDBayer_RGGB    RG
	NDBayer_GBRG    GB
	NDBayer_GRGB    GR
	NDBayer_BGGR    BG
*/


/*
Function that does the barcode decoding. It is passed an image and a vector
that will store all of the codes found in the image. The image is converted to gray,
and a zbar scanner is initialized. The image is changed from an opencv to a Image object,
and then it is scanned by zbar. We then iterate over the discovered symbols in the image, and
create a instance of the struct. the struct is added to the vector, and the bars location
data and type are stored, and printed.
 */
void NDPluginBar::decode_bar_code(Mat &im, vector<bar_QR_code> &codes_in_image){
	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Barcode reader has begun decoding process\n",  driverName, functionName);
	ImageScanner zbarScanner;
	zbarScanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE,1);
	Mat imGray;
	cvtColor(im, imGray, CV_BGR2GRAY);
	Image image(im.cols, im.rows, "Y800", (uchar*) imGray.data, im.cols *im.rows);
	int n = zbarScanner.scan(image);

	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol!=image.symbol_end();++symbol){

		//get information from detected bar code
		bar_QR_code barQR;
		barQR.type = symbol->get_type_name();
		barQR.data = symbol->get_data();

		//cout << "Type : " << barQR.type << endl;
		//cout << "Data : " << barQR.data << endl;
		//set PVs
		setStringParam(NDPluginBarBarcodeType, barQR.type);
		setStringParam(NDPluginBarBarcodeMessage, barQR.data);
		setIntegerParam(NDPluginBarBarcodeFound, 1);
		//push location data
		for(int i = 0; i< symbol->get_location_size(); i++){
			barQR.position.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}
		codes_in_image.push_back(barQR);
	}
}

//uses opencv methods with the locations of the discovered codes to place
//bounding boxes around the areas of the image that contain barcodes. This is
//so the user can confirm that the correct area of the image was discovered
static void show_bar_codes(Mat &im, Mat &result, vector<bar_QR_code> &codes_in_image){
	result = im;
	for(int i = 0; i<codes_in_image.size(); i++){
		vector<Point> barPoints = codes_in_image[i].position;
		vector<Point> outside;
		if(barPoints.size() > 4) convexHull(barPoints, outside);
		else outside = barPoints;
		int n = outside.size();
		for(int j = 0; j<n; j++){
			line(result, outside[j], outside[(j+1)%n], Scalar(0,255,0),3);
		}
	}
	return result;
}


//process callbacks function inherited from NDArray Plugin
void NDPluginBar::processCallbacks(NDArray *pArray){

	//start with an empty array for copy and array info
	NDArray *pScratch = NULL;
	NDArrayInfo arrayInfo;

	//some information we need
	unsigned int numRows, rowSize;
	unsigned char *inData, *outData;
	int barcodes_found;

	static const char* functionName = "processCallbacks";

	// check if image is in mono form
	if (pArray->ndims != 2){
	        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Please convert barcode reader plugin input image format to mono\n",  driverName, functionName);
		return;
	}

	//call base class and get information about frame
	NDPluginDriver::beginProcessCallbacks(pArray);
	pArray->getInfo(&arrayInfo);
	rowSize = pArray->dims[arrayInfo.xDim].size;
	numRows = pArray->dims[arrayInfo.yDim].size;

	getIntegerParam(NDPluginBarBarcodeFound, &barcodes_found);

	if(barcodes_found == 1){
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Please rest the plugin to read additional barcodes\n", driverName, functionName);
		return;
	}

	//unlock the mutex for the processing portion
	this->unlock();

	//create a copy of the array
	NDDimension_t scratch_dims[2];
	pScratch->initDimension(&scratch_dims[0], rowsize);
	pScratch->initDimesion(&scratch_dims[1], numRows);
	this->pNDArrayPool->convert(pArray, &pScratch, NDUInt8);

	pScratch->getInfo(&arrayInfo);
	rowSize = pScratch->dims[arrayInfo.xDim].size;
	numRows = pScratch->dims[arrayInfo.yDim].size;

	//convert the array into an OpenCV "Mat" image
	Mat img = Mat(numRows, rowSize, CV_8UC1);
	Mat barcodeFound;
	vector<bar_QR_code> codes_in_image;

	inData = (unsigned char *)pScratch->pData;
	outData = (unsigned char *)img.data;
	memcpy(outData, inData, arrayInfo.nElements * sizeof(unsigned char));

	//decode the bar codes in the image if any
	decode_bar_code(img, codes_in_image);
	barcodeFound = show_bar_codes(img, barcodeFound, codes_in_image);

	this->lock();

	outData = (unsigned char *) barcodeFound.data;

	//release the array
	if (NULL != pScratch)
		pScratch->release();

	callParamCallbacks();

}

//constructror from base class
NDPluginBar::NDPluginBar(const char *portName, int queueSize, int blockingCallbacks,
		const char *NDArrayPort, int NDArrayAddr,
		int maxBuffers, size_t maxMemory,
		int priority, int stackSize)
		/* Invoke the base class constructor */
		: NDPluginDriver(portName, queueSize, blockingCallbacks,
		NDArrayPort, NDArrayAddr, 1, maxBuffers, maxMemory,
		asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
		asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
		ASYN_MULTIDEVICE, 1, priority, stackSize, 1)
{

	char versionString[25];

	//for debugging
	asynPrint(this->pasynUserSelf, "%s::%s is running\n", driverName, functionName);

	//basic barcode parameters
	createParam(NDPluginBarBarcodeMessageString, asynParamOctet, &NDPluginBarBarcodeMessage);
	createParam(NDPluginBarBarcodeTypeString, asynParamOctet, &NDPluginBarBarcodeType);
	createParam(NDPluginBarBarcodeFoundString, asynParamInt32, &NDPluginBarBarcodeFound);
	createParam(NDPluginBarNumberCodesString, asynParamInt32, &NDPluginBarNumberCodes);

	//x coordinates
	createParam(NDPluginBarUpperLeftXString, asynParamInt32, &NDPluginBarUpperLeftX);
	createParam(NDPluginBarUpperRightXString, asynParamInt32, &NDPluginBarUpperRightX);
	createParam(NDPluginBarLowerLeftXString, asynParamInt32, &NDPluginBarLowerLeftX);
	createParam(NDPluginBarLowerRightXString, asynParamInt32, &NDPluginBarLowerRightX);

	//y coordinates
	createParam(NDPluginBarUpperLeftYString, asynParamInt32, &NDPluginBarUpperLeftY);
	createParam(NDPluginBarUpperRightYString, asynParamInt32, &NDPluginBarUpperRightY);
	createParam(NDPluginBarLowerLeftYString, asynParamInt32, &NDPluginBarLowerLeftY);
	createParam(NDPluginBarLowerRightYString, asynParamInt32, &NDPluginBarLowerRightY);

	setStringParam(NDPluginDriverPluginType, "NDPluginBar");
	epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
	BAR_VERSION, BAR_REVISION, BAR_MODIFICATION);
	setStringParam(NDDriverVersion, versionString);
	connectToArrayPort();
}



extern "C" int NDBarConfigure(const char *portName, int queueSize, int blockingCallbacks,
		const char *NDArrayPort, int NDArrayAddr,
		int maxBuffers, size_t maxMemory,
		int priority, int stackSize){

	NDPluginBar *pPlugin = new NDPluginBar(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
		maxBuffers, maxMemory, priority, stackSize);
	return pPlugin->start();
}

static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "frame queue size",iocshArgInt};
static const iocshArg initArg2 = { "blocking callbacks",iocshArgInt};
static const iocshArg initArg3 = { "NDArrayPort",iocshArgString};
static const iocshArg initArg4 = { "NDArrayAddr",iocshArgInt};
static const iocshArg initArg5 = { "maxBuffers",iocshArgInt};
static const iocshArg initArg6 = { "maxMemory",iocshArgInt};
static const iocshArg initArg7 = { "priority",iocshArgInt};
static const iocshArg initArg8 = { "stackSize",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
					&initArg1,
					&initArg2,
					&initArg3,
					&initArg4,
					&initArg5,
					&initArg6,
					&initArg7,
					&initArg8};

static const iocshFuncDef initFuncDef = {"NDBarConfigure",9,initArgs};

static void initCallFunc(const iocshArgBuf *args){
	NDBarConfigure(args[0].sval, args[1].ival, args[2].ival,
			args[3].sval, args[4].ival, args[5].ival,
			args[6].ival, args[7].ival, args[8].ival);
}


extern "C" void NDBarRegister(void){
	iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
	epicsExportRegistrar(NDBarRegister);
}
