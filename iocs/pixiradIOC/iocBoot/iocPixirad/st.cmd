< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/pixiradApp.dbd")
pixiradApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "13PR1:")
# The drvAsynIPPort port name
epicsEnvSet("COMMAND_PORT", "PIXI_CMD")
# The UDP socket for status
epicsEnvSet("STATUS_PORT", "2224")
# The UDP socket for data.  Pixitrad-1 uses 2223, Pixirad-2 uses 9999
#epicsEnvSet("DATA_PORT", "2223")
epicsEnvSet("DATA_PORT", "9999")
# Number of data port buffers
epicsEnvSet("DATA_PORT_BUFFERS", "1500")
# The port name for the detector
epicsEnvSet("PORT",   "PIXI")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The detector width; used by the driver and for row profiles in the NDPluginStats plugin
#epicsEnvSet("XSIZE",  "476")
epicsEnvSet("XSIZE",  "402")
# The detector height; used by the driver and for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1024")
#epicsEnvSet("YSIZE",  "512")
# The maximum number of time seried points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")


###
# Create the asyn port to talk to the Pixirad box on port 2222.
drvAsynIPPortConfigure("$(COMMAND_PORT)","192.168.0.1:2222 HTTP", 0, 0, 0)
asynOctetSetOutputEos($(COMMAND_PORT), 0, "\n")
asynSetTraceIOMask($(COMMAND_PORT), 0, 2)
#asynSetTraceMask($(COMMAND_PORT), 0, 9)
#asynSetTraceIOTruncateSize($(COMMAND_PORT), 0, 256)
#asynSetTraceFile($(COMMAND_PORT), 0, "asynTrace.txt")

pixiradConfig("$(PORT)", "$(COMMAND_PORT)", "$(DATA_PORT)", "$(STATUS_PORT)", $(DATA_PORT_BUFFERS), $(XSIZE), $(YSIZE))
# The following command is needed for PIII detectors.  
# The values are detector specific and can currently only be found in the Python code that ships with the detector
pixiradAutoCal("$(PORT)", 0, 0, 7, 7, 3, 7, 1850)
asynSetTraceIOMask($(PORT), 0, 2)
#asynSetTraceMask($(PORT), 0, 255)

dbLoadRecords("$(ADPIXIRAD)/db/pixirad.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,SERVER_PORT=$(COMMAND_PORT)")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=487424")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADPIXIRAD)/pixiradApp/Db")

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")

# Force an autocalibration
dbpf("$(PREFIX)cam1:AutoCalibrate", "1")

