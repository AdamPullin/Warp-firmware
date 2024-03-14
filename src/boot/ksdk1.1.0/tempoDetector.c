#include <math.h>
#include <stdlib.h>
 
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "tempoDetector.h"

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

//Square root by newton-raphson iteration, since math.h sqrt causes overflow
uint32_t NRsqrt(uint32_t radicand)
{
    uint32_t sqrt = radicand/10;
    while(1)
    {
        uint32_t nextsqrt = 0.5*(sqrt + radicand/sqrt);
        if (abs(nextsqrt-sqrt) <= 1)
        {
            break;
        }

        sqrt = nextsqrt;
    }
    return sqrt;
}




void tempoAlgorithm(){
    uint16_t xLSB, xMSB, yLSB, yMSB, zLSB, zMSB;
    int16_t xAcc, yAcc, zAcc;
    WarpStatus i2cReadStatus;

    uint16_t filled = 0;


for (int i = 0; i < 1000; i++)
{
    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */);
	xMSB = deviceMMA8451QState.i2cBuffer[0];
	xLSB = deviceMMA8451QState.i2cBuffer[1];
	xAcc = ((xMSB & 0xFF) << 6) | (xLSB >> 2);
    xAcc = (xAcc ^ (1 << 13)) - (1 << 13);

    yMSB = deviceMMA8451QState.i2cBuffer[2];
	yLSB = deviceMMA8451QState.i2cBuffer[3];
	yAcc = ((yMSB & 0xFF) << 6) | (yLSB >> 2);
    yAcc = (yAcc ^ (1 << 13)) - (1 << 13);

    zMSB = deviceMMA8451QState.i2cBuffer[4];
	zLSB = deviceMMA8451QState.i2cBuffer[5];
	zAcc = ((zMSB & 0xFF) << 6) | (zLSB >> 2);
    zAcc = (zAcc ^ (1 << 13)) - (1 << 13);

    uint32_t radicand = ((uint32_t)(xAcc*xAcc) + (uint32_t)(yAcc*yAcc) + (uint32_t)(zAcc*zAcc));
    uint32_t accMagnitude = NRsqrt(radicand);

    for (int n=1; n<25; n++)
    {
        magAccVals[n-1]=magAccVals[n];
        filteredVals[n-1]=filteredVals[n];
        
    }
    magAccVals[24] = accMagnitude;
    filteredVals[24] = 0;
    for (int n=1; n<25; n++)
    {
        filteredVals[24]+=magAccVals[n]*filterCoeffs[n];
    }

    if (filled<26)
    {
        filled++;
        
    }
    else
    {
        if (filteredVals[23]>filteredVals[22] && filteredVals[23]>filteredVals[24])
    {
        if (filteredVals[23] > threshold)
        {
            uint32_t timeNow = OSA_TimeGetMsec();
            warpPrint("timeLOOP %d", timeNow);
            warpPrint("OSAGET %d", OSA_TimeGetMsec);
            warpPrint(" BEAT DETECTED %d: %d 22: %d 24: %d \n",   magAccVals[23], filteredVals[23], filteredVals[22], filteredVals[24]);
            beatCount++;
            if (delayIndex == 0)
            {
                delayIndex++;
                lastBeatTime = timeNow;
                warpPrint(" timenow: %d \n",   timeNow);
            }
            else
            {
                warpPrint(" last time: %d \n",   lastBeatTime);
                beatDelays[delayIndex] = timeNow-lastBeatTime;
                warpPrint(" delay: %d \n",   beatDelays[delayIndex]);
                delayIndex++;
                lastBeatTime = timeNow;
                
            }
        }
    }
    
        warpPrint(" %d: %d \n",   magAccVals[24], filteredVals[24]);

    }

 



}
if (delayIndex > 1)
{
    int averagePeriod;
    for (int j=0; j<delayIndex; j++)
    {
        averagePeriod += beatDelays[j];
        warpPrint(" sum: j= %d  delay= %d \n",   j, beatDelays[j]);
    }
    warpPrint(" sum: %d \n",   averagePeriod);
    averagePeriod /= delayIndex;
    warpPrint(" beat count: %d \n",   beatCount);
    warpPrint(" ave period: %d \n",   averagePeriod);
    int averageTempo = (60*1000)/averagePeriod;
    warpPrint(" ave tempo: %d \n",   averageTempo);
}
else
{
    warpPrint(" not enough beats \n");
}

}