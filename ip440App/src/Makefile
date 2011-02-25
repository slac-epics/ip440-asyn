TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_vxWorks += ip440
LIBRARY_IOC_RTEMS   += ip440

# If your Linux system supports drvIpac and devLib then uncomment this line
LIBRARY_IOC_Linux += ip440

ip440_SRCS += drvIP440.cpp

#INC += drvXXX.h
DBD += ip440.dbd

ip440_LIBS += $(EPICS_BASE_IOC_LIBS)
#=============================


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
