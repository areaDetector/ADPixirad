TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC += pixirad

LIB_SRCS += pixirad.cpp

DBD += pixiradSupport.dbd

PROD_IOC += udp_client_pixirad_single
PROD_SRCS += udp_client_pixirad_single.cpp
PROD_SRCS += PIXIEII_data_utilities.cpp
PROD_LIBS += $(EPICS_BASE_IOC_LIBS)



include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

