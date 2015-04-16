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
# The UDP socket for data
epicsEnvSet("DATA_PORT", "2223")
# Number of data port buffers
epicsEnvSet("DATA_PORT_BUFFERS", "1500")
# The port name for the detector
epicsEnvSet("PORT",   "PIXI")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "476")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "512")
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

pixiradConfig("$(PORT)", "$(COMMAND_PORT)", "$(DATA_PORT)", "$(STATUS_PORT)", $(DATA_PORT_BUFFERS), $(XSIZE), $(YSIZE))
asynSetTraceIOMask($(PORT), 0, 2)
#asynSetTraceMask($(PORT), 0, 255)

dbLoadRecords("$(ADPIXIRAD)/db/pixirad.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,SERVER_PORT=$(COMMAND_PORT)")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=243712")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADPIXIRAD)/pixiradApp/Db")



iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")
