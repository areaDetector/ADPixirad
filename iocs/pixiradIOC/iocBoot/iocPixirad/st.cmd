< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/pixiradApp.dbd")
pixiradApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "13PR1:")
epicsEnvSet("COMMAND_PORT", "PIXI_CMD")
epicsEnvSet("STATUS_PORT", "PIXI_STATUS")
epicsEnvSet("DATA_PORT", "4444")
epicsEnvSet("PORT",   "PIXI")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "512")
epicsEnvSet("YSIZE",  "486")
epicsEnvSet("NCHANS", "2048")

###
# Create the asyn port to talk to the Pixirad box on port 2222.
drvAsynIPPortConfigure("$(COMMAND_PORT)","192.168.0.1:2222", 0, 0, 0)
asynOctetSetOutputEos($(COMMAND_PORT), 0, "\n")
asynOctetSetInputEos($(COMMAND_PORT), 0, "\n")
asynSetTraceIOMask($(COMMAND_PORT), 0, 2)
asynSetTraceMask($(COMMAND_PORT), 0, 9)

###
# Create the asyn port to receive the Pixirad box status broadcasts on port 2224.
drvAsynIPPortConfigure("$(STATUS_PORT)","192.168.0.1:2224", 0, 0, 0)
asynOctetSetOutputEos($(STATUS_PORT), 0, "\n")
asynOctetSetInputEos($(STATUS_PORT), 0, "\n")
asynSetTraceIOMask($(STATUS_PORT), 0, 2)
asynSetTraceMask($(STATUS_PORT), 0, 9)

pixiradConfig("$(PORT)", "$(COMMAND_PORT)", "$(DATA_PORT)", "$(STATUS_PORT)", $(XSIZE), $(YSIZE))
asynSetTraceIOMask($(PORT), 0, 2)
asynSetTraceMask($(PORT), 0, 255)

dbLoadRecords("$(ADCORE)/db/ADBase.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/db/NDFile.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADPIXIRAD)/db/pixirad.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,SERVER_PORT=$(COMMAND_PORT)")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=256000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADPIXIRAD)/pixiradApp/Db")



iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX),D=cam1:")
