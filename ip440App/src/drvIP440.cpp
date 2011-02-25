/* drvIP440.cpp

    Author: Mark Rivers

    This is the driver for the Acromag IP440A digital input IP module
*/

/* EPICS includes */
#include <drvIpac.h>
#include <errlog.h>
#include <epicsTypes.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include <asynPortDriver.h>

#define ACROMAG_ID     0xA3
#define IP440_ID       0x10

#define ALL_BITS 0xFFFFFFFF

#define MAX_MESSAGES 1000

typedef struct {
    volatile epicsUInt8 *inputPort0;
    volatile epicsUInt8 *inputPort1;
    volatile epicsUInt8 *inputPort2;
    volatile epicsUInt8 *inputPort3;
} IP440Registers;

static const char *driverName = "IP440";

/** This is the class definition for the IP440 class*/
class IP440 : public asynPortDriver
{
public:
    IP440(const char *portName, int carrier, int slot, int msecPoll);
    virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask);
    virtual void report(FILE *fp, int details);
    // These should be private, but are called from C, so must be public
    void pollerThread();  

private:
    epicsUInt8 *baseAddress;
    IP440Registers regs;
    epicsUInt32 prevValue;
    double pollTime;
    int initialized;
    int dataParam;
};

// These functions must have C linkage because they are called from other EPICS components
extern "C" {
static void pollerThreadC(void * pPvt)
{
    IP440 *pIP440 = (IP440 *)pPvt;
    pIP440->pollerThread();
}}

IP440::IP440(const char *portName, int carrier, int slot, int msecPoll)
    :asynPortDriver(portName,1,1,             // portName, maxAddr, paramTableSize
                    asynUInt32DigitalMask,    // interfaceMask
                    asynUInt32DigitalMask,    // interruptMask
                    0,1,0,0)                  // asynFlags, autoConnect, priority, stackSize
{
    //static const char *functionName = "IP440";
    ipac_idProm_t *id;
    epicsUInt8 *base;
    int manufacturer, model;
   
    this->pollTime = msecPoll / 1000.;
    
    this->initialized = 0;
   
    if (ipmCheck(carrier, slot)) {
       errlogPrintf("%s: bad carrier or slot\n", driverName);
       return;
    }
    id = (ipac_idProm_t *) ipmBaseAddr(carrier, slot, ipac_addrID);
    base = (epicsUInt8 *) ipmBaseAddr(carrier, slot, ipac_addrIO);
    this->baseAddress = base;
    manufacturer = id->manufacturerId & 0xff;
    model = id->modelId & 0xff;

    if ((manufacturer != ACROMAG_ID) || ( model != IP440_ID)) {
        errlogPrintf("%s: manufacturer and/or model incorrect = %x/%x, should be %x/%x\n",
            driverName, manufacturer, model, ACROMAG_ID, IP440_ID);
        return;
    }

    /* Set up the register pointers.*/
    /* Define registers in units of 8-bit bytes */
    this->regs.inputPort0 = base + 0x1;
    this->regs.inputPort1 = base + 0x3;
    this->regs.inputPort2 = base + 0x5;
    this->regs.inputPort3 = base + 0x7;
    
    /* Create the asynPortDriver parameter for the data */
    createParam("DIGITAL_DATA", asynParamUInt32Digital, &this->dataParam); 

    this->initialized = 1;

    /* Start the thread to poll and handle interrupt callbacks to 
     * device support */
    epicsThreadCreate("IP440",
                      epicsThreadPriorityHigh,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)pollerThreadC,
                      this);
}

    
asynStatus IP440::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask)
{
    static const char *functionName = "readUInt32Digital";
    IP440Registers r = this->regs;

    if (!this->initialized) return(asynError);
    *value  = *r.inputPort0;
    *value |= ((*r.inputPort1) << 8);
    *value |= ((*r.inputPort2) << 16);
    *value |= ((*r.inputPort3) << 24);
    *value &= mask;
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s:, *value=%x\n", 
              driverName, functionName, *value);
    return(asynSuccess);
}

void IP440::pollerThread()
{
    /* This function runs in a separate thread.  It waits for the poll time.  
     * If the bits read from the IP440 have changed then it does callbacks to all clients that
     * have registered with registerDevCallback */
    static const char *functionName = "pollerThread";
    epicsUInt32 newValue, changedBits;
    int firstTime=1;

    while(1) {      
        /*  Wait for an interrupt or for the poll time, whichever comes first */
        epicsThreadSleep(pollTime);
        readUInt32Digital(this->pasynUserSelf, &newValue, ALL_BITS);
        asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
                  "%s:%s:, bits=%x, this->oldBits=%x\n", 
                  driverName, functionName, newValue, this->prevValue);

        changedBits = newValue ^ this->prevValue;
        if (changedBits || firstTime) {
            firstTime = 0;
            this->prevValue = newValue;
            setUIntDigitalParam(dataParam, newValue, ALL_BITS);
            setInterruptUInt32Digital(this->pasynUserSelf, changedBits, interruptOnBoth);
            callParamCallbacks();
        }
    }
}

void IP440::report(FILE *fp, int details)
{
    epicsUInt32 value;

    if (!this->initialized) {
        fprintf(fp, "%s %s: not initialized!\n", driverName, this->portName);
        return;
    }
    fprintf(fp, "%s %s: connected at base address %p\n",
            driverName, this->portName, this->baseAddress);
    if (details >= 1) {
        getUIntDigitalParam(dataParam, &value, ALL_BITS);
        fprintf(fp, "  current value=%x\n", value);
    }
    asynPortDriver::report(fp, details);
}

extern "C" int initIP440(const char *portName, int carrier, int slot, int msecPoll)
{
    IP440 *pIP440 = new IP440(portName, carrier, slot, msecPoll);
    pIP440 = NULL;
    return(asynSuccess);
}

/* iocsh functions */
static const iocshArg initArg0 = { "Port name",iocshArgString};
static const iocshArg initArg1 = { "Carrier",iocshArgInt};
static const iocshArg initArg2 = { "Slot",iocshArgInt};
static const iocshArg initArg3 = { "msecPoll",iocshArgInt};
static const iocshArg * const initArgs[4] = {&initArg0,
                                             &initArg1,
                                             &initArg2,
                                             &initArg3};

static const iocshFuncDef initFuncDef = {"initIP440",4,initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    initIP440(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}

void IP440Register(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(IP440Register);






