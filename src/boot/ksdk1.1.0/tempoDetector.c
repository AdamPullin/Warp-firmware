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
    int32_t starttime;
    int32_t endtime;
    uint16_t filled = 0;



for (int i = 0; i < 1000; i++)
{

    starttime = OSA_TimeGetMsec();
    

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
            //warpPrint(" BEAT DETECTED  \n");
            beatCount++;
            if (delayIndex == 0)
            {
                delayIndex++;
                lastBeatTime = timeNow;
            }
            else
            {
                beatDelays[delayIndex-1] = timeNow-lastBeatTime;
                delayIndex++;
                lastBeatTime = timeNow;
                
            }
        }
    }
    
        //warpPrint(" %d: %d \n",   magAccVals[24], filteredVals[24]);
        warpPrint(" end time: %d \n",   (OSA_TimeGetMsec()-starttime));


    }

}
if (beatCount > 1)
{
    //int averagePeriod;
    //for (int j=0; j<delayIndex; j++)
    //{
    //    averagePeriod += beatDelays[j];
    //}
    //averagePeriod /= (beatCount-1);
    //warpPrint(" beat count: %d \n",   beatCount);
    //warpPrint(" ave period: %d \n",   averagePeriod);
    //int averageTempo = (60*1000)/averagePeriod;
    //warpPrint(" ave tempo: %d \n",   averageTempo);
    for (int j=0; j<delayIndex-1; j++)
    {
        int delayIn = beatDelays[j];
        int bpmIn = 60000/delayIn;
        if (delayIn > 865)
        {
            bins[0] += 400;
        }
        else if (delayIn < 392 && delayIn > 0)
        {
            warpPrint("%d speedy, \n", delayIn);
            bins[9] += 400;
        }
        else if (delayIn > 0)
        {
            int binIndex = (bpmIn/10) -6;

            //check lower BPM threshold
            if (bin_thresholds[binIndex] - delayIn  < 9 )
            {
                int add = 200+(bin_thresholds[binIndex] - delayIn)* 25;
                bins[binIndex] += add;
                bins[binIndex-1] += (400-add);
                
            }

            //check upper BPM threshold
            else if (delayIn - bin_thresholds[binIndex+1] < 9) 
            {
                int add = 200 + (delayIn - bin_thresholds[binIndex+1])*25;
                bins[binIndex] += add;
                bins[binIndex+1] += (400-add);
            }

            //otherwise securely in current bin
            else
            {
                warpPrint("%d is securely in bin %d, \n", delayIn, binIndex);
                bins[binIndex]+= 400;
            }

        }
         warpPrint(" ncompile test \n");
    }

    //ensure percentage probabilities add up to 100 (rounding errors)
    int dividedsum = 0;
    int divisor = (4*(delayIndex-1));
    int counter = 1;
    
    for(int p = 0; p < 10; p++)
    {
        dividedsum += (bins[p])/divisor;
        binremainders[p] = (bins[p]) % divisor;
        bins[p] = bins[p]/divisor;
        //warpPrint(" bin %d: %d \n",p, ((bins[p])/divisor));
    }
    while (dividedsum != 100)
    {
        if (dividedsum<100)
        {
            dividedsum=0;
            for (int k = 0; k<10; k++)
            {
                if (binremainders[k] == divisor-1)
                {
                    bins[k]++;
                    binremainders[k]=0;
                    break;
                }
                
            }    
            divisor--;
        } 

        if (dividedsum>100)
        {
            dividedsum=0;
            for (int k = 0; k<10; k++)
            {
                if (binremainders[k] == counter)
                {
                    bins[k]--;
                    binremainders[k]=1000;
                    break;
                }
            }
            counter++;
        } 

        for (int k = 0; k<10; k++)
            {
                dividedsum+=bins[k];
            }
    }
    
    for(int p = 0; p < 10; p++)
    {
        warpPrint(" bin %d: %d \n",p, ((bins[p])));
    }
    // make percentages add to 100: eg scale by beatcount; int biggest = 0; for i in bins, sum += i, ; if i>biggest biggest = i, i+= 100 - sum; 

}
else
{
    warpPrint(" not enough beats \n");
}

}