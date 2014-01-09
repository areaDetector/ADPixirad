/* pixirad.cpp
 *
 * This is a driver for the PiXurad pixel array detector.
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  January 8, 2014
 *
 */
 
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

/** Messages to/from server */
#define MAX_MESSAGE_SIZE 256 
#define MAX_FILENAME_LEN 256
#define MAX_HEADER_STRING_LEN 68
/** Time to poll when reading from server */
#define ASYN_POLL_TIME .01 
#define SERVER_DEFAULT_TIMEOUT 1.0
/** Additional time to wait for a server response after the acquire should be complete */ 
#define SERVER_ACQUIRE_TIMEOUT 10.
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

/** Trigger modes */
typedef enum {
    TMInternal,
    TMExternalEnable,
    TMExternalTrigger,
    TMMultipleExternalTrigger,
    TMAlignment
} PixiradTriggerMode;

static const char *driverName = "pixirad";

#define PixiradDelayTimeString      "DELAY_TIME"
#define PixiradThresholdString      "THRESHOLD"
#define PixiradArmedString          "ARMED"
#define PixiradImageFileTmotString  "IMAGE_FILE_TMOT"
#define PixiradFlatFieldFileString  "FLAT_FIELD_FILE"
#define PixiradMinFlatFieldString   "MIN_FLAT_FIELD"
#define PixiradFlatFieldValidString "FLAT_FIELD_VALID"
#define PixiradThTemp0String        "TH_TEMP_0"
#define PixiradThTemp1String        "TH_TEMP_1"
#define PixiradThTemp2String        "TH_TEMP_2"
#define PixiradThHumid0String       "TH_HUMID_0"
#define PixiradThHumid1String       "TH_HUMID_1"
#define PixiradThHumid2String       "TH_HUMID_2"


/** Driver for PiXirad pixel array detectors using their server server over TCP/IP socket */
class pixirad : public ADDriver {
public:
    pixirad(const char *portName, const char *serverPort,
                    int maxSizeX, int maxSizeY,
                    int maxBuffers, size_t maxMemory,
                    int priority, int stackSize);
                 
    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual);
    void report(FILE *fp, int details);
    /* These should be private but are called from C so must be public */
    void pixiradTask(); 
    
protected:
    int PixiradDelayTime;
    #define FIRST_PIXIRAD_PARAM PixiradDelayTime
    int PixiradThreshold;
    int PixiradImageFileTmot;
    int PixiradFlatFieldFile;
    int PixiradMinFlatField;
    int PixiradFlatFieldValid;
    int PixiradThTemp0;
    int PixiradThTemp1;
    int PixiradThTemp2;
    int PixiradThHumid0;
    int PixiradThHumid1;
    int PixiradThHumid2;
    #define LAST_PIXIRAD_PARAM PixiradThHumid2

private:                                       
    /* These are the methods that are new to this class */
    void abortAcquisition();
    void makeMultipleFileFormat(const char *baseFileName);
    asynStatus waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    int stringEndsWith(const char *aString, const char *aSubstring, int shouldIgnoreCase);
    asynStatus readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus readTiff(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus writeServer(double timeout);
    asynStatus readServer(double timeout);
    asynStatus writeReadServer(double timeout);
    asynStatus setAcquireParams();
    asynStatus setThreshold();
    asynStatus pixiradStatus();
    void readFlatFieldFile(const char *flatFieldFile);
   
    /* Our data */
    int imagesRemaining;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    char toServer[MAX_MESSAGE_SIZE];
    char fromServer[MAX_MESSAGE_SIZE];
    NDArray *pFlatField;
    char multipleFileFormat[MAX_FILENAME_LEN];
    int multipleFileNumber;
    asynUser *pasynUserServer;
    double averageFlatField;
    double demandedThreshold;
    int firstStatusCall;
};

#define NUM_PIXIRAD_PARAMS ((int)(&LAST_PIXIRAD_PARAM - &FIRST_PIXIRAD_PARAM + 1))

static void pixiradTaskC(void *drvPvt)
{
    pixirad *pPvt = (pixirad *)drvPvt;
    
    pPvt->pixiradTask();
}


/** Constructor for Pixirad driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] serverPort The name of the asyn port previously created with drvAsynIPPortConfigure to
  *            communicate with the PiXirad server.
  * \param[in] maxSizeX The size of the Pixirad detector in the X direction.
  * \param[in] maxSizeY The size of the Pixirad detector in the Y direction.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
pixirad::pixirad(const char *portName, const char *serverPort,
                                int maxSizeX, int maxSizeY,
                                int maxBuffers, size_t maxMemory,
                                int priority, int stackSize)

    : ADDriver(portName, 1, NUM_PIXIRAD_PARAMS, maxBuffers, maxMemory,
               0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
      imagesRemaining(0), firstStatusCall(1)

{
    int status = asynSuccess;
    const char *functionName = "pixirad";
    size_t dims[2];

    /* Create the epicsEvents for signaling to the Pixirad task when acquisition starts and stops */
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId) {
        printf("%s:%s epicsEventCreate failure for start event\n", 
            driverName, functionName);
        return;
    }
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId) {
        printf("%s:%s epicsEventCreate failure for stop event\n", 
            driverName, functionName);
        return;
    }
    
    /* Allocate the raw buffer we use to read image files.  Only do this once */
    dims[0] = maxSizeX;
    dims[1] = maxSizeY;
    /* Allocate the raw buffer we use for flat fields. */
    this->pFlatField = this->pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);
    
    /* Connect to Pixirad server */
    status = pasynOctetSyncIO->connect(serverPort, 0, &this->pasynUserServer, NULL);

    createParam(PixiradDelayTimeString,      asynParamFloat64, &PixiradDelayTime);
    createParam(PixiradThresholdString,      asynParamFloat64, &PixiradThreshold);
    createParam(PixiradImageFileTmotString,  asynParamFloat64, &PixiradImageFileTmot);
    createParam(PixiradFlatFieldFileString,  asynParamOctet,   &PixiradFlatFieldFile);
    createParam(PixiradMinFlatFieldString,   asynParamInt32,   &PixiradMinFlatField);
    createParam(PixiradFlatFieldValidString, asynParamInt32,   &PixiradFlatFieldValid);
    createParam(PixiradThTemp0String,        asynParamFloat64, &PixiradThTemp0);
    createParam(PixiradThTemp1String,        asynParamFloat64, &PixiradThTemp1);
    createParam(PixiradThTemp2String,        asynParamFloat64, &PixiradThTemp2);
    createParam(PixiradThHumid0String,       asynParamFloat64, &PixiradThHumid0);
    createParam(PixiradThHumid1String,       asynParamFloat64, &PixiradThHumid1);
    createParam(PixiradThHumid2String,       asynParamFloat64, &PixiradThHumid2);

    /* Set some default values for parameters */
    status =  setStringParam (ADManufacturer, "Pixirad");
    status |= setStringParam (ADModel, "Pixirad 1");
    status |= setIntegerParam(ADMaxSizeX, maxSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxSizeY);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeY, maxSizeY);
    status |= setIntegerParam(NDArraySizeX, maxSizeX);
    status |= setIntegerParam(NDArraySizeY, maxSizeY);
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType,  NDUInt32);
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setIntegerParam(ADTriggerMode, TMInternal);

    status |= setStringParam (PixiradFlatFieldFile, "");
    status |= setIntegerParam(PixiradFlatFieldValid, 0);

    setDoubleParam(PixiradThTemp0, 0);
    setDoubleParam(PixiradThTemp1, 0);
    setDoubleParam(PixiradThTemp2, 0);
    setDoubleParam(PixiradThHumid0, 0);
    setDoubleParam(PixiradThHumid1, 0);
    setDoubleParam(PixiradThHumid2, 0);

    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }
    
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("PixiradTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)pixiradTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image task\n", 
            driverName, functionName);
        return;
    }

}



void pixirad::readFlatFieldFile(const char *flatFieldFile)
{
    size_t i;
    int status;
    int ngood;
    int minFlatField;
    epicsInt32 *pData;
    const char *functionName = "readFlatFieldFile";
    NDArrayInfo arrayInfo;
    
    setIntegerParam(PixiradFlatFieldValid, 0);
    this->pFlatField->getInfo(&arrayInfo);
    getIntegerParam(PixiradMinFlatField, &minFlatField);
    if (strlen(flatFieldFile) == 0) return;
    status = readImageFile(flatFieldFile, NULL, 0., this->pFlatField);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, error reading flat field file %s\n",
            driverName, functionName, flatFieldFile);
        return;
    }
    /* Compute the average counts in the flat field */
    this->averageFlatField = 0.;
    ngood = 0;
    
    for (i=0, pData = (epicsInt32 *)this->pFlatField->pData; 
         i<arrayInfo.nElements; 
         i++, pData++) {
        if (*pData < minFlatField) continue;
        ngood++;
        averageFlatField += *pData;
    }
    averageFlatField = averageFlatField/ngood;
    
    for (i=0, pData = (epicsInt32 *)this->pFlatField->pData; 
         i<arrayInfo.nElements; 
         i++, pData++) {
        if (*pData < minFlatField) *pData = (epicsInt32)averageFlatField;
    }
    /* Call the NDArray callback */
    /* Must release the lock here, or we can get into a deadlock, because we can
     * block on the plugin lock, and the plugin can be calling us */
    this->unlock();
    doCallbacksGenericPointer(this->pFlatField, NDArrayData, 0);
    this->lock();
    setIntegerParam(PixiradFlatFieldValid, 1);
}


/** This function waits for the specified file to exist.  It checks to make sure that
 * the creation time of the file is after a start time passed to it, to force it to wait
 * for a new file to be created.
 */
asynStatus pixirad::waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    int fd=-1;
    int fileExists=0;
    struct stat statBuff;
    epicsTimeStamp tStart, tCheck;
    time_t acqStartTime;
    double deltaTime=0.;
    int status=-1;
    const char *functionName = "waitForFileToExist";

    if (pStartTime) epicsTimeToTime_t(&acqStartTime, pStartTime);
    epicsTimeGetCurrent(&tStart);

    while (deltaTime <= timeout) {
        fd = open(fileName, O_RDONLY, 0);
        if ((fd >= 0) && (timeout != 0.)) {
            fileExists = 1;
            /* The file exists.  Make sure it is a new file, not an old one.
             * We don't do this check if timeout==0, which is used for reading flat field files */
            status = fstat(fd, &statBuff);
            if (status){
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s error calling fstat, errno=%d %s\n",
                    driverName, functionName, errno, fileName);
                close(fd);
                return(asynError);
            }
            /* We allow up to 10 second clock skew between time on machine running this IOC
             * and the machine with the file system returning modification time */
            if (difftime(statBuff.st_mtime, acqStartTime) > -10) break;
            close(fd);
            fd = -1;
        }
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        unlock();
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        lock();
        if (status == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }
    if (fd < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s timeout waiting for file to be created %s\n",
            driverName, functionName, fileName);
        if (fileExists) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "  file exists but is more than 10 seconds old, possible clock synchronization problem\n");
            setStringParam(ADStatusMessage, "Image file is more than 10 seconds old");
        } else
            setStringParam(ADStatusMessage, "Timeout waiting for file to be created");
        return(asynError);
    }
    close(fd);
    return(asynSuccess);
}

/** This function reads TIFF or CBF image files.  It is not intended to be general, it
 * is intended to read the TIFF or CBF files that the server creates.  It checks to make
 * sure that the creation time of the file is after a start time passed to it, to force
 * it to wait for a new file to be created.
 */
asynStatus pixirad::readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    const char *functionName = "readImageFile";

    if (stringEndsWith(fileName, ".tif", 1) || stringEndsWith(fileName, ".tiff", 1)) {
        return readTiff(fileName, pStartTime, timeout, pImage);
    } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, unsupported image file name extension, expected .tif or .cbf, fileName=%s\n",
            driverName, functionName, fileName);
        setStringParam(ADStatusMessage, "Unsupported file extension, expected .tif or .cbf");
        return(asynError);
    }
}

/** This function reads TIFF files using libTiff.  It is not intended to be general,
 * it is intended to read the TIFF files that the server creates.  It checks to make sure
 * that the creation time of the file is after a start time passed to it, to force it to
 * wait for a new file to be created.
 */
asynStatus pixirad::readTiff(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    int status=-1;
    const char *functionName = "readTiff";
    size_t totalSize;
    int size;
    int numStrips, strip;
    char *buffer;
    TIFF *tiff=NULL;
    epicsUInt32 uval;

    deltaTime = 0.;
    epicsTimeGetCurrent(&tStart);

    /* Suppress error messages from the TIFF library */
    TIFFSetErrorHandler(NULL);
    TIFFSetWarningHandler(NULL);

    status = waitForFileToExist(fileName, pStartTime, timeout, pImage);
    if (status != asynSuccess) {
        return((asynStatus)status);
    }

    while (deltaTime <= timeout) {
        /* At this point we know the file exists, but it may not be completely written yet.
         * If we get errors then try again */
        tiff = TIFFOpen(fileName, "rc");
        if (tiff == NULL) {
            status = asynError;
            goto retry;
        }
        
        /* Do some basic checking that the image size is what we expect */
        status = TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &uval);
        if (uval != (epicsUInt32)pImage->dims[0].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image width incorrect =%u, should be %u\n",
                driverName, functionName, uval, (epicsUInt32)pImage->dims[0].size);
            goto retry;
        }
        status = TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &uval);
        if (uval != (epicsUInt32)pImage->dims[1].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image length incorrect =%u, should be %u\n",
                driverName, functionName, uval, (epicsUInt32)pImage->dims[1].size);
            goto retry;
        }
        numStrips= TIFFNumberOfStrips(tiff);
        buffer = (char *)pImage->pData;
        totalSize = 0;
        for (strip=0; (strip < numStrips) && (totalSize < pImage->dataSize); strip++) {
            size = TIFFReadEncodedStrip(tiff, 0, buffer, pImage->dataSize-totalSize);
            if (size == -1) {
                /* There was an error reading the file.  Most commonly this is because the file
                 * was not yet completely written.  Try again. */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s, error reading TIFF file %s\n",
                    driverName, functionName, fileName);
                goto retry;
            }
            buffer += size;
            totalSize += size;
        }
        if (totalSize != pImage->dataSize) {
            status = asynError;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, file size incorrect =%lu, should be %lu\n",
                driverName, functionName, (unsigned long)totalSize, (unsigned long)pImage->dataSize);
            goto retry;
        }
        /* Sucesss! */
        break;
        
        retry:
        if (tiff != NULL) TIFFClose(tiff);
        tiff = NULL;
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        unlock();
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        lock();
        if (status == epicsEventWaitOK) {
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

    if (tiff != NULL) TIFFClose(tiff);

    return(asynSuccess);
}   

asynStatus pixirad::setAcquireParams()
{
    int ival;
    double dval;
    int triggerMode;
    asynStatus status;
    char *substr = NULL;
    int pixelCutOff = 0;
    
    status = getIntegerParam(ADTriggerMode, &triggerMode);
    if (status != asynSuccess) triggerMode = TMInternal;
    
     /* When we change modes download all exposure parameters, since some modes
      * replace values with new parameters */
    if (triggerMode == TMAlignment) {
        setIntegerParam(ADNumImages, 1);
    }
    
    status = getIntegerParam(ADNumImages, &ival);
    if ((status != asynSuccess) || (ival < 1)) {
        ival = 1;
        setIntegerParam(ADNumImages, ival);
    }
    epicsSnprintf(this->toServer, sizeof(this->toServer), "nimages %d", ival);
    writeReadServer(SERVER_DEFAULT_TIMEOUT); 

    status = getIntegerParam(ADNumExposures, &ival);
    if ((status != asynSuccess) || (ival < 1)) {
        ival = 1;
        setIntegerParam(ADNumExposures, ival);
    }
    epicsSnprintf(this->toServer, sizeof(this->toServer), "nexpframe %d", ival);
    writeReadServer(SERVER_DEFAULT_TIMEOUT); 

    status = getDoubleParam(ADAcquireTime, &dval);
    if ((status != asynSuccess) || (dval < 0.)) {
        dval = 1.;
        setDoubleParam(ADAcquireTime, dval);
    }
    epicsSnprintf(this->toServer, sizeof(this->toServer), "exptime %f", dval);
    writeReadServer(SERVER_DEFAULT_TIMEOUT);

    status = getDoubleParam(ADAcquirePeriod, &dval);
    if ((status != asynSuccess) || (dval < 0.)) {
        dval = 2.;
        setDoubleParam(ADAcquirePeriod, dval);
    }
    epicsSnprintf(this->toServer, sizeof(this->toServer), "expperiod %f", dval);
    writeReadServer(SERVER_DEFAULT_TIMEOUT);

    status = getDoubleParam(PixiradDelayTime, &dval);
    if ((status != asynSuccess) || (dval < 0.)) {
        dval = 0.;
        setDoubleParam(PixiradDelayTime, dval);
    }
    epicsSnprintf(this->toServer, sizeof(this->toServer), "delay %f", dval);
    writeReadServer(SERVER_DEFAULT_TIMEOUT);

   
    return(asynSuccess);

}

asynStatus pixirad::setThreshold()
{
    int igain, status;
    double threshold, dgain;
    char *substr = NULL;
    int threshold_readback = 0;
    
    getDoubleParam(ADGain, &dgain);
    igain = (int)(dgain + 0.5);
    if (igain < 0) igain = 0;
    if (igain > 3) igain = 3;
    threshold = this->demandedThreshold;
    epicsSnprintf(this->toServer, sizeof(this->toServer), "SetThreshold %s %f", 
                    gainStrings[igain], threshold*1000.);
    /* Set the status to waiting so we can be notified when it has finished */
    setIntegerParam(ADStatus, ADStatusWaiting);
    setStringParam(ADStatusMessage, "Setting threshold");
    callParamCallbacks();
    
    status=writeReadServer(110.0);  /* This command can take 96 seconds on a 6M */
    if (status)
        setIntegerParam(ADStatus, ADStatusError);
    else
        setIntegerParam(ADStatus, ADStatusIdle);
    setIntegerParam(PixiradThresholdApply, 0);

    /* Read back the actual setting, in case we are out of bounds. */
    epicsSnprintf(this->toServer, sizeof(this->toServer), "SetThreshold");
    status=writeReadServer(5.0); 

    /* Response should contain "threshold: 9000 eV; vcmp:"*/
    if (!status) {
        if ((substr = strstr(this->fromServer, "threshold: ")) != NULL) {
            sscanf(strtok(substr, ";"), "threshold: %d eV", &threshold_readback);
            setDoubleParam(PixiradThreshold, (double)threshold_readback/1000.0);
        }
    }
    

    /* The SetThreshold command resets numimages to 1 and gapfill to 0, so re-send current
     * acquisition parameters */
    setAcquireParams();

    callParamCallbacks();

    return(asynSuccess);
}

asynStatus pixirad::writeServer(double timeout)
{
    size_t nwrite;
    asynStatus status;
    const char *functionName="writeServer";

    /* Flush any stale input, since the next operation is likely to be a read */
    status = pasynOctetSyncIO->flush(this->pasynUserServer);
    status = pasynOctetSyncIO->write(this->pasynUserServer, this->toServer,
                                     strlen(this->toServer), timeout,
                                     &nwrite);
                                        
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s, status=%d, sent\n%s\n",
                    driverName, functionName, status, this->toServer);

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringToServer, this->toServer);
    
    return(status);
}


asynStatus pixirad::readServer(double timeout)
{
    size_t nread;
    asynStatus status=asynSuccess;
    int eventStatus;
    asynUser *pasynUser = this->pasynUserServer;
    int eomReason;
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    const char *functionName="readServer";

    /* We implement the timeout with a loop so that the port does not
     * block during the entire read.  If we don't do this then it is not possible
     * to abort a long exposure */
    deltaTime = 0;
    epicsTimeGetCurrent(&tStart);
    while (deltaTime <= timeout) {
        unlock();
        status = pasynOctetSyncIO->read(pasynUser, this->fromServer,
                                        sizeof(this->fromServer), ASYN_POLL_TIME,
                                        &nread, &eomReason);
        /* Check for an abort event sent during a read. Otherwise we can miss it and mess up the next acqusition.*/
        eventStatus = epicsEventWaitWithTimeout(this->stopEventId, 0.001);
        lock();
        if (eventStatus == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        if (status != asynTimeout) break;

        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        unlock();
        eventStatus = epicsEventWaitWithTimeout(this->stopEventId, ASYN_POLL_TIME);
        lock();
        if (eventStatus == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

    // If we got asynTimeout, and timeout=0 then this is not an error, it is a poll checking for possible reply and we are done
   if ((status == asynTimeout) && (timeout == 0)) return(asynSuccess);
   if (status != asynSuccess)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, timeout=%f, status=%d received %lu bytes\n%s\n",
                    driverName, functionName, timeout, status, (unsigned long)nread, this->fromServer);
   else {
        /* Look for the string OK in the response */
        if (!strstr(this->fromServer, "OK")) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s unexpected response from server, no OK, response=%s\n",
                      driverName, functionName, this->fromServer);
            setStringParam(ADStatusMessage, "Error from server");
            status = asynError;
        } else
            setStringParam(ADStatusMessage, "Server returned OK");
    }

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringFromServer, this->fromServer);

    return(status);
}

asynStatus pixirad::writeReadServer(double timeout)
{
    asynStatus status;
    
    status = writeServer(timeout);
    if (status) return status;
    status = readServer(timeout);
    return status;
}

static void pixiradTaskC(void *drvPvt)
{
    pixirad *pPvt = (pixirad *)drvPvt;
    
    pPvt->pixiradTask();
}

/** This thread controls acquisition, reads image files to get the image data, and
  * does the callbacks to send it to higher layers */
void pixirad::pixiradTask()
{
    int status = asynSuccess;
    int imageCounter;
    int numImages;
    int numExposures;
    int multipleFileNextImage=0;  /* This is the next image number, starting at 0 */
    int acquire;
    ADStatus_t acquiring;
    NDArray *pImage;
    double acquireTime, acquirePeriod;
    double readImageFileTimeout, timeout;
    int triggerMode;
    epicsTimeStamp startTime;
    const char *functionName = "pixiradTask";
    char fullFileName[MAX_FILENAME_LEN];
    char filePath[MAX_FILENAME_LEN];
    char statusMessage[MAX_MESSAGE_SIZE];
    size_t dims[2];
    int itemp;
    int arrayCallbacks;
    int flatFieldValid;
    int aborted = 0;
    int statusParam = 0;

    this->lock();

    /* Loop forever */
    while (1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if ((aborted) || (!acquire)) {
            /* Only set the status message if we didn't encounter any errors last time, so we don't overwrite the 
             error message */
            if (!status)
            setStringParam(ADStatusMessage, "Waiting for acquire command");
            callParamCallbacks();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            this->unlock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
            status = epicsEventWait(this->startEventId);
            this->lock();
            aborted = 0;
            acquire = 1;
        }
        
        /* We are acquiring. */
        /* Get the current time */
        epicsTimeGetCurrent(&startTime);
        
        /* Get the exposure parameters */
        getDoubleParam(ADAcquireTime, &acquireTime);
        getDoubleParam(ADAcquirePeriod, &acquirePeriod);
        getDoubleParam(PixiradImageFileTmot, &readImageFileTimeout);
        
        /* Get the acquisition parameters */
        getIntegerParam(ADTriggerMode, &triggerMode);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumExposures, &numExposures);
        
        acquiring = ADStatusAcquire;
        setIntegerParam(ADStatus, acquiring);

        /* Create the full filename */
        createFileName(sizeof(fullFileName), fullFileName);
        
        switch (triggerMode) {
            case TMInternal:
                epicsSnprintf(this->toServer, sizeof(this->toServer), 
                    "Exposure %s", fullFileName);
                break;
            case TMExternalEnable:
                epicsSnprintf(this->toServer, sizeof(this->toServer), 
                    "ExtEnable %s", fullFileName);
                break;
            case TMExternalTrigger:
                epicsSnprintf(this->toServer, sizeof(this->toServer), 
                    "ExtTrigger %s", fullFileName);
                break;
            case TMMultipleExternalTrigger:
                epicsSnprintf(this->toServer, sizeof(this->toServer), 
                    "ExtMTrigger %s", fullFileName);
                break;
            case TMAlignment:
                getStringParam(NDFilePath, sizeof(filePath), filePath);
                epicsSnprintf(fullFileName, sizeof(fullFileName), "%salignment.tif", 
                              filePath);
                epicsSnprintf(this->toServer, sizeof(this->toServer), 
                    "Exposure %s", fullFileName);
                break;
        }
        setStringParam(ADStatusMessage, "Starting exposure");
        /* Send the acquire command to server and wait for the 15OK response */
        writeReadServer(2.0);
        /* Do another read in case there is an ERR string at the end of the input buffer */
        status=readServer(0.0);

        /* If the status wasn't asynSuccess or asynTimeout, report the error */
        if (status>1) {
            acquire = 0;
        }
        else {
            /* Set status back to asynSuccess as the timeout was expected */
            status = asynSuccess;
            /* Open the shutter */
            setShutter(1);
            /* Set the armed flag */
            setIntegerParam(PixiradArmed, 1);
            /* Create the format string for constructing file names for multi-image collection */
            makeMultipleFileFormat(fullFileName);
            multipleFileNextImage = 0;
            /* Call the callbacks to update any changes */
            setStringParam(NDFullFileName, fullFileName);
            callParamCallbacks();
        }

        while (acquire) {
            if (numImages == 1) {
                /* For single frame or alignment mode need to wait for 7OK response from server
                 * saying acquisition is complete before trying to read file, else we get a
                 * recent but stale file. */
                setStringParam(ADStatusMessage, "Waiting for 7OK response");
                callParamCallbacks();
                timeout = ((numExposures-1) * acquirePeriod) + acquireTime;
                status = readServer(timeout + SERVER_ACQUIRE_TIMEOUT);
                /* If there was an error jump to bottom of loop */
                if (status) {
                    acquire = 0;
                    aborted = 1;
                    if(status==asynTimeout) {
                        setStringParam(ADStatusMessage, "Timeout waiting for server response");
                        epicsSnprintf(this->toServer, sizeof(this->toServer), "Stop");
                        writeReadServer(SERVER_DEFAULT_TIMEOUT);
                        epicsSnprintf(this->toServer, sizeof(this->toServer), "K");
                        writeServer(SERVER_DEFAULT_TIMEOUT);
                    }
                    continue;
                }
            } else {
                /* If this is a multi-file acquisition the file name is built differently */
                epicsSnprintf(fullFileName, sizeof(fullFileName), multipleFileFormat, 
                              multipleFileNumber);
                setStringParam(NDFullFileName, fullFileName);
            }
            getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

            if (arrayCallbacks) {
                getIntegerParam(NDArrayCounter, &imageCounter);
                imageCounter++;
                setIntegerParam(NDArrayCounter, imageCounter);
                /* Call the callbacks to update any changes */
                callParamCallbacks();

                /* Get an image buffer from the pool */
                getIntegerParam(ADMaxSizeX, &itemp); dims[0] = itemp;
                getIntegerParam(ADMaxSizeY, &itemp); dims[1] = itemp;
                pImage = this->pNDArrayPool->alloc(2, dims, NDInt32, 0, NULL);
                epicsSnprintf(statusMessage, sizeof(statusMessage), "Reading image file %s", fullFileName);
                setStringParam(ADStatusMessage, statusMessage);
                callParamCallbacks();
                /* We release the mutex when calling readImageFile, because this takes a long time and
                 * we need to allow abort operations to get through */
                status = readImageFile(fullFileName, &startTime, 
                                       (numExposures * acquireTime) + readImageFileTimeout, 
                                       pImage); 
                /* If there was an error jump to bottom of loop */
                if (status) {
                    acquire = 0;
                    aborted = 1;
                    pImage->release();
                    continue;
                }

                getIntegerParam(PixiradFlatFieldValid, &flatFieldValid);
                if (flatFieldValid) {
                    epicsInt32 *pData, *pFlat;
                    size_t i;
                    for (i=0, pData = (epicsInt32 *)pImage->pData, pFlat = (epicsInt32 *)this->pFlatField->pData;
                         i<dims[0]*dims[1]; 
                         i++, pData++, pFlat++) {
                        *pData = (epicsInt32)((this->averageFlatField * *pData) / *pFlat);
                    }
                } 
                /* Put the frame number and time stamp into the buffer */
                pImage->uniqueId = imageCounter;
                pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
                updateTimeStamp(&pImage->epicsTS);

                /* Get any attributes that have been defined for this driver */        
                this->getAttributes(pImage->pAttributeList);
                
                /* Call the NDArray callback */
                /* Must release the lock here, or we can get into a deadlock, because we can
                 * block on the plugin lock, and the plugin can be calling us */
                this->unlock();
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                     "%s:%s: calling NDArray callback\n", driverName, functionName);
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                this->lock();
                /* Free the image buffer */
                pImage->release();
            }
            if (numImages == 1) {
                if (triggerMode == TMAlignment) {
                    epicsSnprintf(this->toServer, sizeof(this->toServer), 
                        "Exposure %s", fullFileName);
                    /* Send the acquire command to server and wait for the 15OK response */
                    writeReadServer(2.0);
                } else {
                    acquire = 0;
                }
            } else if (numImages > 1) {
                multipleFileNextImage++;
                multipleFileNumber++;
                if (multipleFileNextImage == numImages) acquire = 0;
            }
            
        }
        /* We are done acquiring */
        /* Wait for the 7OK response from server in the case of multiple images */
        if ((numImages > 1) && (status == asynSuccess)) {
            /* If arrayCallbacks is 0 we will have gone through the above loop without waiting
             * for each image file to be written.  Thus, we may need to wait a long time for
             * the 7OK response.  
             * If arrayCallbacks is 1 then the response should arrive fairly soon. */
            if (arrayCallbacks) 
                timeout = readImageFileTimeout;
            else 
                timeout = (numImages * numExposures * acquirePeriod) + SERVER_ACQUIRE_TIMEOUT;
            setStringParam(ADStatusMessage, "Waiting for 7OK response");
            callParamCallbacks();
            status = readServer(timeout);
            /* In the case of a timeout, server could still be acquiring. So we need to send a stop.*/
            if (status == asynTimeout) {
                setStringParam(ADStatusMessage, "Timeout waiting for server response");
                epicsSnprintf(this->toServer, sizeof(this->toServer), "Stop");
                writeReadServer(SERVER_DEFAULT_TIMEOUT);
                epicsSnprintf(this->toServer, sizeof(this->toServer), "K");
                writeServer(SERVER_DEFAULT_TIMEOUT);
                aborted = 1;
            }
        }

        /* If everything was ok, set the status back to idle */
        getIntegerParam(ADStatus, &statusParam);
        if (!status) {
            setIntegerParam(ADStatus, ADStatusIdle);
        } else {
            if (statusParam != ADStatusAborted) {
                setIntegerParam(ADStatus, ADStatusError);
            }
        }

        /* Call the callbacks to update any changes */
        callParamCallbacks();

        setShutter(0);
        setIntegerParam(ADAcquire, 0);
        setIntegerParam(PixiradArmed, 0);

        /* Call the callbacks to update any changes */
        callParamCallbacks();        
    }
}

/** This function is called periodically read the detector status (temperature, humidity, etc.)
    It should not be called if we are acquiring data, to avoid polling server when taking data.*/
asynStatus pixirad::pixiradStatus()
{
  asynStatus status = asynSuccess;
  float temp = 0.0;
  float humid = 0.0;
  char *substr = NULL;
  char *substrtok = NULL;

  /* Read temp and humidity.*/
  epicsSnprintf(this->toServer, sizeof(this->toServer), "Th");
  status=writeReadServer(1.0); 

  /* Response should contain: 
     Channel 0: Temperature = 31.4C, Rel. Humidity = 22.1%;\n
     Channel 1: Temperature = 25.8C, Rel. Humidity = 33.5%;\n
     Channel 2: Temperature = 28.6C, Rel. Humidity = 2.0%
     However, not every detector has all 3 channels.*/

  if (!status) {

    if ((substr = strstr(strtok(this->fromServer, "%"), "Channel 0")) != NULL) {
      sscanf(substr, "Channel 0: Temperature = %fC, Rel. Humidity = %f", &temp, &humid);
      setDoubleParam(PixiradThTemp0, temp);
      setDoubleParam(PixiradThHumid0, humid);
      setDoubleParam(ADTemperature, temp);
    }
    if ((substrtok = strtok(NULL, "%")) != NULL) {
      if ((substr = strstr(substrtok, "Channel 1")) != NULL) {
        sscanf(substr, "Channel 1: Temperature = %fC, Rel. Humidity = %f", &temp, &humid);
        setDoubleParam(PixiradThTemp1, temp);
        setDoubleParam(PixiradThHumid1, humid);
      }
    }
    if ((substrtok = strtok(NULL, "%")) != NULL) {
      if ((substr = strstr(substrtok, "Channel 2")) != NULL) {
        sscanf(substr, "Channel 2: Temperature = %fC, Rel. Humidity = %f", &temp, &humid);
        setDoubleParam(PixiradThTemp2, temp);
        setDoubleParam(PixiradThHumid2, humid);
      }
    }

  } else {
    setIntegerParam(ADStatus, ADStatusError);
  }      
  callParamCallbacks();
  return status;
}



/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADTriggerMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus pixirad::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32";

    /* Ensure that ADStatus is set correctly before we set ADAcquire.*/
    getIntegerParam(ADStatus, &adstatus);
    if (function == ADAcquire) {
      if (value && ((adstatus == ADStatusIdle) || adstatus == ADStatusError || adstatus == ADStatusAborted)) {
        setStringParam(ADStatusMessage, "Acquiring data");
        setIntegerParam(ADStatus, ADStatusAcquire);
      }
      if (!value && (adstatus == ADStatusAcquire)) {
        setStringParam(ADStatusMessage, "Acquisition aborted");
        setIntegerParam(ADStatus, ADStatusAborted);
      }
    }
    callParamCallbacks();

    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value && (adstatus == ADStatusIdle || adstatus == ADStatusError || adstatus == ADStatusAborted)) {
            /* Send an event to wake up the Pixirad task.  */
            epicsEventSignal(this->startEventId);
        } 
        if (!value && (adstatus == ADStatusAcquire)) {
          /* This was a command to stop acquisition */
            epicsEventSignal(this->stopEventId);
            epicsSnprintf(this->toServer, sizeof(this->toServer), "Stop");
            writeReadServer(SERVER_DEFAULT_TIMEOUT);
            epicsSnprintf(this->toServer, sizeof(this->toServer), "K");
            writeServer(SERVER_DEFAULT_TIMEOUT);
            /* Sleep for two seconds to allow acqusition to stop in server.*/
            epicsThreadSleep(2);
            setStringParam(ADStatusMessage, "Acquisition aborted");
        }
    } else if ((function == ADTriggerMode) ||
               (function == ADNumImages) ||
               (function == ADNumExposures)) {
        setAcquireParams();
   } else if (function == ADReadStatus) {
        if (adstatus != ADStatusAcquire) {
          status = pixiradStatus();
        }
    } else { 
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PIXIRAD_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }
            
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}


/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus pixirad::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeFloat64";
    double oldValue;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    getDoubleParam(function, &oldValue);
    status = setDoubleParam(function, value);
    

    /* Changing any of the following parameters requires recomputing the base image */
    if ((function == ADGain) ||
        (function == PixiradThreshold)) {
        if (function == PixiradThreshold) {
            this->demandedThreshold = value;
        }
        if (function == PixiradThreshold) {
          status = setDoubleParam(function, this->demandedThreshold);
        }
        setThreshold();
    } else if ((function == ADAcquireTime) ||
               (function == ADAcquirePeriod) ||
               (function == PixiradDelayTime)) {
        setAcquireParams();
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PIXIRAD_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }

    if (status) {
        /* Something went wrong so we set the old value back */
        setDoubleParam(function, oldValue);
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    }
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including ADFilePath, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus pixirad::writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeOctet";

    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, (char *)value);

    if (function == PixiradFlatFieldFile) {
        this->readFlatFieldFile(value);
    } else if (function == NDFilePath) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), "imgpath %s", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);
        this->checkPath();
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PIXIRAD_PARAM) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
    }
    
     /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();

    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, value=%s", 
                  driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%s\n", 
              driverName, functionName, function, value);
    *nActual = nChars;
    return status;
}



/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void pixirad::report(FILE *fp, int details)
{

    fprintf(fp, "Pixirad detector %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

extern "C" int pixiradConfig(const char *portName, const char *serverPort, 
                                    int maxSizeX, int maxSizeY,
                                    int maxBuffers, size_t maxMemory,
                                    int priority, int stackSize)
{
    new pixirad(portName, serverPort, maxSizeX, maxSizeY, maxBuffers, maxMemory,
                        priority, stackSize);
    return(asynSuccess);
}

/* Code for iocsh registration */
static const iocshArg pixiradConfigArg0 = {"Port name", iocshArgString};
static const iocshArg pixiradConfigArg1 = {"server port name", iocshArgString};
static const iocshArg pixiradConfigArg2 = {"maxSizeX", iocshArgInt};
static const iocshArg pixiradConfigArg3 = {"maxSizeY", iocshArgInt};
static const iocshArg pixiradConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg pixiradConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg pixiradConfigArg6 = {"priority", iocshArgInt};
static const iocshArg pixiradConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const pixiradConfigArgs[] =  {&pixiradConfigArg0,
                                                              &pixiradConfigArg1,
                                                              &pixiradConfigArg2,
                                                              &pixiradConfigArg3,
                                                              &pixiradConfigArg4,
                                                              &pixiradConfigArg5,
                                                              &pixiradConfigArg6,
                                                              &pixiradConfigArg7};
static const iocshFuncDef configpixirad = {"pixiradConfig", 8, pixiradConfigArgs};
static void configpixiradCallFunc(const iocshArgBuf *args)
{
    pixiradConfig(args[0].sval, args[1].sval, args[2].ival,  args[3].ival,  
                          args[4].ival, args[5].ival, args[6].ival,  args[7].ival);
}


static void pixiradRegister(void)
{

    iocshRegister(&configpixirad, configpixiradCallFunc);
}

extern "C" {
epicsExportRegistrar(pixiradRegister);
}

