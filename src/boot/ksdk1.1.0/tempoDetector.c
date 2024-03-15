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

//Square root by newton iteration, since math.h sqrt causes overflow
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
    int32_t inittime = OSA_TimeGetMsec();



while (OSA_TimeGetMsec() - inittime < 10001)
{    
    // Read in next samples
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

    // Pythagorean theorem to find magnitude 
    uint32_t radicand = ((uint32_t)(xAcc*xAcc) + (uint32_t)(yAcc*yAcc) + (uint32_t)(zAcc*zAcc));
    uint32_t accMagnitude = NRsqrt(radicand);

    // Shift buffers to make space for new values, n=1 to overwrite oldest value
    for (int n=1; n<25; n++)
    {
        magAccVals[n-1]=magAccVals[n];
        filteredVals[n-1]=filteredVals[n];
        
    }

    // Write new magnitude value into buffer
    magAccVals[24] = accMagnitude;

    // Set end value of filtered buffer to zero, calculate filtered value for new magnitude buffer, write
    filteredVals[24] = 0;
    for (int n=1; n<25; n++)
    {
        filteredVals[24]+=magAccVals[n]*filterCoeffs[n];
    }


    // Make sure buffer has been filled with samples before continuing
    if (filled<26)
    {
        filled++;
        
    }
    else
    {
        // Check if local maximum
        if (filteredVals[23]>filteredVals[22] && filteredVals[23]>filteredVals[24])
    {
        // Check if above threshold
        if (filteredVals[23] > threshold)
        {
            // Record current time stamp
            uint32_t timeNow = OSA_TimeGetMsec();
            warpPrint(" BEAT DETECTED  \n");

            // Add to beat count
            beatCount++;
            // If first detected beat just store
            if (delayIndex == 0)
            {
                delayIndex++;
                lastBeatTime = timeNow;
            }
            // If not first detected beat, subtract from last detected beat, store period between beats in beatDelays
            else
            {
                beatDelays[delayIndex-1] = timeNow-lastBeatTime;
                delayIndex++;
                lastBeatTime = timeNow;
                
            }
        }
    }

        //warpPrint(" %d: %d \n",   magAccVals[24], filteredVals[24]);
    }
}

// SAMPLING STAGE OVER, START PROCESSING

// Only process if more than one beat detected
if (beatCount > 1)
{
    // Iterate through delays, calculate implied BPMs
    for (int j=0; j<delayIndex-1; j++)
    {
        int delayIn = beatDelays[j];
        int bpmIn = 60000/delayIn;

        //Above threshold plus uncertainty for lowest tempo
        if (delayIn > 865)
        {
            bins[0] += 400;
        }

        //Below threshold minus uncertainty for highest tempo
        else if (delayIn < 392 && delayIn > 0)
        {
            bins[9] += 400;
        }

        else if (delayIn > 0)
        {
            // Find bin by truncating, -6 removes the offset from first bin being up to 70 bpm
            int binIndex = (bpmIn/10) -6;

            //check lower BPM threshold
            if (bin_thresholds[binIndex] - delayIn  < 9 )
            {
                // Spread points depending on distance from boundary
                int add = 200+(bin_thresholds[binIndex] - delayIn)* 25;
                bins[binIndex] += add;
                bins[binIndex-1] += (400-add);
                
            }

            //check upper BPM threshold
            else if (delayIn - bin_thresholds[binIndex+1] < 9) 
            {
                // Spread points depending on distance from boundary
                int add = 200 + (delayIn - bin_thresholds[binIndex+1])*25;
                bins[binIndex] += add;
                bins[binIndex+1] += (400-add);
            }

            //otherwise securely in current bin
            else
            {
                //Allocate all points to one bin
                bins[binIndex]+= 400;
            }

        }
    }

    //Ensure percentage probabilities add up to 100 (after truncation)
    int dividedsum = 0;
    int divisor = (4*(delayIndex-1));
    int counter = 1;
    
    for(int p = 0; p < 10; p++)
    {
        //Find initial sum of percentages
        dividedsum += (bins[p])/divisor;

        //Store remainders from initial division
        binremainders[p] = (bins[p]) % divisor;

        //Scale bin values to be percentages
        bins[p] = bins[p]/divisor;
    }


    while (dividedsum != 100)
    {
        //If sum to less than 100, find the highest value that was truncated and increment it
        //Repeat for next highest value etc until sum = 100
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

        //If sum to more than 100, find the lowest value that was truncated and decrement it
        //Repeat for next lowest value etc until sum = 100
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

        //Recalculate percentage sum
        for (int k = 0; k<10; k++)
            {
                dividedsum+=bins[k];
            }
    }

    //Output calculated percentage probabilities    
    warpPrint("       Tempo range          %% probability \n");
    warpPrint("        T <70 BPM:            %d %% \n", bins[0]);
    warpPrint("  70 <= T <80 BPM:            %d %% \n", bins[1]);
    warpPrint("  80 <= T <90 BPM:            %d %% \n", bins[2]);
    warpPrint(" 90 <= T <100 BPM:            %d %% \n", bins[3]);
    warpPrint("100 <= T <110 BPM:            %d %% \n", bins[4]);
    warpPrint("110 <= T <120 BPM:            %d %% \n", bins[5]);
    warpPrint("120 <= T <130 BPM:            %d %% \n", bins[6]);
    warpPrint("130 <= T <140 BPM:            %d %% \n", bins[7]);
    warpPrint("140 <= T <150 BPM:            %d %% \n", bins[8]);
    warpPrint("     T >= 150 BPM:            %d %% \n", bins[9]);

    
    
}

//If fewer than two beats, can't estimate a tempo
else
{
    warpPrint(" not enough beats \n");
}

}
