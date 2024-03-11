#include <math.h>
 */
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

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;




void tempoAlgorithm(){
    uint16_t xLSB, xMSB, yLSB, yMSB, zLSB, zMSB;
    int16_t xAcc, yAcc, zAcc;
    WarpStatus i2cReadStatus;


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

    uint32_t accMagnitude = sqrt((uint32_t)(xAcc*xAcc) + (uint32_t)(yAcc*yAcc) + (uint32_t)(zAcc*zAcc));
    
}