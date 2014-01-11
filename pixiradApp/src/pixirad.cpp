/* pixirad.cpp
 *
 * This is a driver for the PiXirad pixel array detector.
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

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

/** Messages to/from server */
#define MAX_MESSAGE_SIZE 256 
#define MAX_FILENAME_LEN 256
/** Time to poll when reading from server */
#define ASYN_POLL_TIME .01 
#define SERVER_DEFAULT_TIMEOUT 1.0
/** Additional time to wait for a server response after the acquire should be complete */ 
#define SERVER_ACQUIRE_TIMEOUT 10.
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

/** Save data */
typedef enum {
    SaveDataOff,
    SaveDataOn
} PixiradSaveDataState_t;
static const char *PixiradSaveDataStateStrings[] = {"OFF", "ON"};

/** Trigger modes */
typedef enum {
    TMInternal,
    TMExternal,
    TMBulb
} PixiradTriggerMode_t;
static const char *PixiradTriggerModeStrings[] = {"INT", "EXT1", "EXT2"};

/** Cooling state */
typedef enum {
    CoolingOff,
    CoolingOn
} PixiradCoolingState_t;
static const char *PixiradCoolingStateStrings[] = {"OFF", "ON"};

/** High voltage state */
typedef enum {
    HVOff,
    HVOn
} PixiradHVState_t;
static const char *PixiradHVStateStrings[] = {"OFF", "ON"};

/** High voltage mode */
typedef enum {
    HVManual,
    HVAuto
} PixiradHVMode_t;
static const char *PixiradHVModeStrings[] = {"MANUAL", "AUTO"};

/** Sync polarity */
typedef enum {
    SyncPos,
    SyncNeg
} PixiradSyncPolarity_t;
static const char *PixiradSyncPolarityStrings[] = {"POS", "NEG"};

/** Download speed */
typedef enum {
    SpeedHigh,
    SpeedLow
} PixiradDownloadSpeed_t;
static const char *PixiradDownloadSpeedStrings[] = {"UNMOD", "MOD"};

/** Collection mode */
typedef enum {
    CMOneColorLow,
    CMOneColorHigh,
    CMTwoColors,
    CMFourColors,
    CMOneColorDTF,
    CMTwoColorsDTF
} PixiradCollectionMode_t;
static const char *PixiradCollectionModeStrings[] = {"1COL0", "1COL1", "2COL", "4COL", "DTF", "2COLDTF"};

static const char *driverName = "pixirad";

#define PixiradCollectionModeString  "COLLECTION_MODE"
#define PixiradSaveDataString        "SAVE_DATA"
#define PixiradThreshold1String      "THRESHOLD1"
#define PixiradThreshold2String      "THRESHOLD2"
#define PixiradThreshold3String      "THRESHOLD3"
#define PixiradThreshold4String      "THRESHOLD4"
#define PixiradHVValueString         "HV_VALUE"
#define PixiradHVStateString         "HV_STATE"
#define PixiradHVModeString          "HV_MODE"
#define PixiradHVDelayString         "HV_DELAY"
#define PixiradSyncInPolarityString  "SYNC_IN_POLARITY"
#define PixiradSyncOutPolarityString "SYNC_OUT_POLARITY"
#define PixiradDownloadSpeedString   "DOWNLOAD_SPEED"
#define PixiradImageFileTmotString   "IMAGE_FILE_TMOT"
#define PixiradCoolingStateString    "COOLING_STATE"
#define PixiradHotTemperatureString  "HOT_TEMPERATURE"
#define PixiradBoxTemperatureString  "BOX_TEMPERATURE"
#define PixiradBoxHumidityString     "BOX_HUMIDITY"
#define PixiradPeltierPowerString    "PELTIER_POWER"


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
    void report(FILE *fp, int details);
    /* These should be private but are called from C so must be public */
    void pixiradTask(); 
    
protected:
    int PixiradCollectionMode;
    #define FIRST_PIXIRAD_PARAM PixiradCollectionMode
    int PixiradSaveData;
    int PixiradThreshold1;
    int PixiradThreshold2;
    int PixiradThreshold3;
    int PixiradThreshold4;
    int PixiradHVValue;
    int PixiradHVState;
    int PixiradHVMode;
    int PixiradHVDelay;
    int PixiradSyncInPolarity;
    int PixiradSyncOutPolarity;
    int PixiradDownloadSpeed;
    int PixiradImageFileTmot;
    int PixiradCoolingState;
    int PixiradHotTemperature;
    int PixiradBoxTemperature;
    int PixiradBoxHumidity;
    int PixiradPeltierPower;
    #define LAST_PIXIRAD_PARAM PixiradPeltierPower

private:                                       
    /* These are the methods that are new to this class */
    asynStatus waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout);
    asynStatus readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus writeReadServer(double timeout);
    asynStatus readServer(double timeout);
    asynStatus setAcquireTimes();
    asynStatus pixiradStatus();
    void pixiradReadFileTask(); 
   
    /* Our data */
    int imagesRemaining;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    char toServer[MAX_MESSAGE_SIZE];
    char fromServer[MAX_MESSAGE_SIZE];
    asynUser *pasynUserServer;
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
      imagesRemaining(0)

{
    int status = asynSuccess;
    const char *functionName = "pixirad";

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
    
    /* Connect to Pixirad server */
    status = pasynOctetSyncIO->connect(serverPort, 0, &this->pasynUserServer, NULL);

    createParam(PixiradCollectionModeString,  asynParamInt32,   &PixiradCollectionMode);
    createParam(PixiradSaveDataString,        asynParamInt32,   &PixiradSaveData);
    createParam(PixiradThreshold1String,      asynParamFloat64, &PixiradThreshold1);
    createParam(PixiradThreshold2String,      asynParamFloat64, &PixiradThreshold2);
    createParam(PixiradThreshold3String,      asynParamFloat64, &PixiradThreshold3);
    createParam(PixiradThreshold4String,      asynParamFloat64, &PixiradThreshold4);
    createParam(PixiradHVValueString,         asynParamFloat64, &PixiradHVValue);
    createParam(PixiradHVStateString,         asynParamInt32,   &PixiradHVState);
    createParam(PixiradHVModeString,          asynParamInt32,   &PixiradHVMode);
    createParam(PixiradHVDelayString,         asynParamFloat64, &PixiradHVDelay);
    createParam(PixiradSyncInPolarityString,  asynParamInt32,   &PixiradSyncInPolarity);
    createParam(PixiradSyncOutPolarityString, asynParamInt32,   &PixiradSyncOutPolarity);
    createParam(PixiradDownloadSpeedString,   asynParamInt32,   &PixiradDownloadSpeed);
    createParam(PixiradImageFileTmotString,   asynParamFloat64, &PixiradImageFileTmot);
    createParam(PixiradCoolingStateString,    asynParamInt32,   &PixiradCoolingState);
    createParam(PixiradHotTemperatureString,  asynParamFloat64, &PixiradHotTemperature);
    createParam(PixiradBoxTemperatureString,  asynParamFloat64, &PixiradBoxTemperature);
    createParam(PixiradBoxHumidityString,     asynParamFloat64, &PixiradBoxHumidity);
    createParam(PixiradPeltierPowerString,    asynParamFloat64, &PixiradPeltierPower);

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
    status |= setDoubleParam(ADAcquireTime, 1.0);

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


/** This function waits for the specified file to exist.  It checks to make sure that
 * the creation time of the file is after a start time passed to it, to force it to wait
 * for a new file to be created.
 */
asynStatus pixirad::waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout)
{
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
        status = stat(fileName, &statBuff);
        if (status == 0) {
            fileExists = 1;
            /* The file exists.  Make sure it is a new file, not an old one.
             * We allow up to 10 second clock skew between time on machine running this IOC
             * and the machine with the file system returning modification time */
            if (difftime(statBuff.st_mtime, acqStartTime) > -10) return asynSuccess;
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
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s::%s timeout waiting for file to be created %s\n",
        driverName, functionName, fileName);
    if (fileExists) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "  file exists but is more than 10 seconds old, possible clock synchronization problem\n");
        setStringParam(ADStatusMessage, "Image file is more than 10 seconds old");
    } else
        setStringParam(ADStatusMessage, "Timeout waiting for file to be created");
    return asynError;
}

/** This function reads TIFF or CBF image files.  It is not intended to be general, it
 * is intended to read the TIFF or CBF files that the server creates.  It checks to make
 * sure that the creation time of the file is after a start time passed to it, to force
 * it to wait for a new file to be created.
 */
asynStatus pixirad::readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    const char *functionName = "readImageFile";
    epicsTimeStamp tStart, tCheck;
    struct stat statBuff;
    double deltaTime;
    FILE *file;
    int status=-1;
    size_t totalSize;
    int size;
    char *buffer;
    epicsUInt32 uval;

    deltaTime = 0.;
    epicsTimeGetCurrent(&tStart);

    status = waitForFileToExist(fileName, pStartTime, timeout);
    if (status != asynSuccess) {
        return((asynStatus)status);
    }

    while (deltaTime <= timeout) {
        /* At this point we know the file exists, but it may not be completely written yet.
         * If we get errors then try again */
        file = fopen(fileName, "r");
        if (file == NULL) {
            status = asynError;
        }
        
        /* Do some basic checking that the image size is what we expect */

        buffer = (char *)pImage->pData;
        totalSize = 0;
        /* Sucesss! */
        break;
        
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

    if (file != NULL) fclose(file);

    return(asynSuccess);
}   

asynStatus pixirad::setAcquireTimes()
{
    double acquireTime;
    double acquirePeriod;
    double shutterPause;
    asynStatus status;
    
    getDoubleParam(ADAcquireTime, &acquireTime);
    getDoubleParam(ADAcquirePeriod, &acquirePeriod);
    
    epicsSnprintf(this->toServer, sizeof(this->toServer), 
                  "SET_SHUTTER_WIDTH %f", acquireTime*1000.);
    writeReadServer(SERVER_DEFAULT_TIMEOUT);
    shutterPause = acquirePeriod - acquireTime;
    if (shutterPause < 0.) shutterPause = 0;
    epicsSnprintf(this->toServer, sizeof(this->toServer), 
                  "SET_SHUTTER_PAUSE %f", shutterPause*1000.);
    status = writeReadServer(SERVER_DEFAULT_TIMEOUT);
    return status;
}

asynStatus pixirad::writeReadServer(double timeout)
{
    size_t nwrite, nread;
    asynStatus status=asynSuccess;
    asynUser *pasynUser = this->pasynUserServer;
    int eomReason;
    const char *functionName="writeReadServer";

    status = pasynOctetSyncIO->writeRead(pasynUser, this->toServer, strlen(this->toServer), 
                                         this->fromServer, sizeof(this->fromServer), 
                                         timeout, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, timeout=%f, status=%d wrote %lu bytes, received %lu bytes\n%s\n",
                    driverName, functionName, timeout, status, (unsigned long)nwrite, (unsigned long)nread, this->fromServer);
    else {
        /* Look for the string ": DONE" in the response */
        if (!strstr(this->fromServer, ": DONE")) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s unexpected response from server = %s\n",
                      driverName, functionName, this->fromServer);
            setStringParam(ADStatusMessage, "Error from server");
            status = asynError;
        } else
            setStringParam(ADStatusMessage, "Server returned OK");
    }

    /* Set input and output strings so they can be displayed by EPICS */
    setStringParam(ADStringToServer, this->toServer);
    setStringParam(ADStringFromServer, this->fromServer);

    return(status);
}

asynStatus pixirad::readServer(double timeout)
{
    size_t nread;
    asynStatus status=asynSuccess;
    asynUser *pasynUser = this->pasynUserServer;
    int eomReason;
    const char *functionName="writeReadServer";

    status = pasynOctetSyncIO->read(pasynUser, this->fromServer, sizeof(this->fromServer), 
                                         timeout, &nread, &eomReason);
    if (status != asynSuccess)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, timeout=%f, status=%d received %lu bytes\n%s\n",
                    driverName, functionName, timeout, status, (unsigned long)nread, this->fromServer);

    /* Set output string so it can be displayed by EPICS */
    setStringParam(ADStringFromServer, this->fromServer);

    return(status);
}

/** This thread controls acquisition, reads image files to get the image data, and
  * does the callbacks to send it to higher layers */
void pixirad::pixiradTask()
{
    int status = asynSuccess;
    int numImages;
    int acquire;
    ADStatus_t acquiring;
    double acquireTime, acquirePeriod;
    double readImageFileTimeout, timeout;
    int triggerMode;
    epicsTimeStamp startTime;
    char fullFileName[MAX_FILENAME_LEN];
    int aborted = 0;
    int statusParam = 0;
    static const char *functionName = "pixiradTask";

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
        
        acquiring = ADStatusAcquire;
        setIntegerParam(ADStatus, acquiring);

        /* Create the full filename */
        createFileName(sizeof(fullFileName), fullFileName);
        setStringParam(NDFullFileName, fullFileName);
        
         /* Send the file path to server */
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_FILE_PATH %s", fullFileName);
        writeReadServer(1.0);
        
        setStringParam(ADStatusMessage, "Starting exposure");
        /* Send the acquire command to server */
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "START_ACQUISITION");
        writeReadServer(1.0);

        /* Open the shutter */
        setShutter(1);
        callParamCallbacks();
        
        // This thread just waits for the ACQUISITION_DONE reply
        

        setStringParam(ADStatusMessage, "Waiting for ACQUISITION_DONE response");
        callParamCallbacks();
        timeout = (numImages * acquirePeriod) + SERVER_ACQUIRE_TIMEOUT;
        status = readServer(timeout);
        if (status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error wating for ACQUISITION_DONE response from server, status=%d\n",
                driverName, functionName, status);
            if(status==asynTimeout) {
                setStringParam(ADStatusMessage, "Timeout waiting for server response");
                epicsSnprintf(this->toServer, sizeof(this->toServer), "STOP_ACQUISITION");
                writeReadServer(SERVER_DEFAULT_TIMEOUT);
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

        /* Call the callbacks to update any changes */
        callParamCallbacks();        
    }
}

// This thread reads the image file 
void pixirad::pixiradReadFileTask()
{
    int status = asynSuccess;
    int arrayCallbacks;
    NDArray *pImage;
    int imageCounter;
    int itemp;
    size_t dims[2];
    epicsTimeStamp startTime;
    double readImageFileTimeout;
    double acquireTime;
    char fullFileName[MAX_FILENAME_LEN];
    char statusMessage[MAX_MESSAGE_SIZE];
    static const char *functionName = "pixiradReadFileTask";

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
                               acquireTime + readImageFileTimeout, 
                               pImage); 

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
}

/** This function is called periodically read the environmental parameters (temperature, humidity, etc.)
    It should not be called if we are acquiring data, to avoid polling server when taking data.*/
asynStatus pixirad::pixiradStatus()
{
  asynStatus status = asynSuccess;
  int numItems;
  float value;

  /* Read cold temperature */
  epicsSnprintf(this->toServer, sizeof(this->toServer), "READ_COLD_TEMP");
  status=writeReadServer(1.0);
  if (status) return status;
  numItems = sscanf(this->fromServer, "READ_COLD_TEMP: DONE: %f", &value);
  if (numItems != 1) return asynError;
  setDoubleParam(ADTemperatureActual, value);

  /* Read hot temperature */
  epicsSnprintf(this->toServer, sizeof(this->toServer), "READ_HOT_TEMP");
  status=writeReadServer(1.0);
  if (status) return status;
  numItems = sscanf(this->fromServer, "READ_HOT_TEMP: DONE: %f", &value);
  if (numItems != 1) return asynError;
  setDoubleParam(PixiradHotTemperature, value);

  /* Read box temperature */
  epicsSnprintf(this->toServer, sizeof(this->toServer), "READ_BOX_TEMP");
  status=writeReadServer(1.0);
  if (status) return status;
  numItems = sscanf(this->fromServer, "READ_BOX_TEMP: DONE: %f", &value);
  if (numItems != 1) return asynError;
  setDoubleParam(PixiradBoxTemperature, value);

  /* Read box humidity */
  epicsSnprintf(this->toServer, sizeof(this->toServer), "READ_BOX_HUM");
  status=writeReadServer(1.0);
  if (status) return status;
  numItems = sscanf(this->fromServer, "READ_BOX_HUM: DONE: %f", &value);
  if (numItems != 1) return asynError;
  setDoubleParam(PixiradBoxHumidity, value);

  /* Read Peltier power */
  epicsSnprintf(this->toServer, sizeof(this->toServer), "READ_PELTIER_PWR");
  status=writeReadServer(1.0);
  if (status) return status;
  numItems = sscanf(this->fromServer, "READ_PELTIER_PWR: DONE: %f", &value);
  if (numItems != 1) return asynError;
  setDoubleParam(PixiradPeltierPower, value);

  callParamCallbacks();
  return asynSuccess;
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
            epicsSnprintf(this->toServer, sizeof(this->toServer), "STOP_ACQUISITION");
            writeReadServer(SERVER_DEFAULT_TIMEOUT);
            /* Sleep for two seconds to allow acqusition to stop in server.*/
            epicsThreadSleep(2);
            setStringParam(ADStatusMessage, "Acquisition aborted");
        }

    } else if (function == ADTriggerMode) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_TRIGGER_SOURCE %s", PixiradTriggerModeStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == ADNumImages) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_FRAME %d", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradSaveData) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_REC %s", PixiradSaveDataStateStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradCollectionMode) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_MODALITY %s", PixiradCollectionModeStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradSyncInPolarity) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_SYNC_IN_POL %s", PixiradSyncPolarityStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradDownloadSpeed) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_DOWNLOAD_MODE %s", PixiradDownloadSpeedStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradSyncOutPolarity) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_SYNC_OUT_POL %s", PixiradSyncPolarityStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradHVMode) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_HV_MANAGEMENT %s", PixiradHVModeStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradHVState) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_HV_STATE %s", PixiradHVStateStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradCoolingState) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_COOLING_STATE %s", PixiradCoolingStateStrings[value]);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

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

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);
    
    if ((function == ADAcquireTime) ||
        (function == ADAcquirePeriod)) {
      status = setAcquireTimes();

    } else if (function == ADTemperature) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_COOLING_VALUE %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradHVValue) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_HV_VALUE %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradHVDelay) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_HV_DELAY %f", value*1000.);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradThreshold1) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_THRESHOLD %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_FIRST_THRESHOLD %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradThreshold2) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_SECOND_THRESHOLD %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradThreshold3) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_THIRD_THRESHOLD %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else if (function == PixiradThreshold4) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), 
                      "SET_FOURTH_THRESHOLD %f", value);
        writeReadServer(SERVER_DEFAULT_TIMEOUT);

    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PIXIRAD_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }

    if (status) {
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

