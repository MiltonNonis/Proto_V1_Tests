/*****************************************************************************
 *  File name : SensorUtils.c
 *  Desc      : Nuzzle Collar Sensor Driver support utility functions.
 *              This driver code implementation is generic for all sensors on Collar.
 *  Dates : 03/22/15 - 04/13/15
 *        - Milton Nonis
 * Copyright (c) Nuzzle, Inc. 2015
 * All rights reserved. This code is Livescribe, Inc proprietary and confidential, 
 * and may be protected by patents. Any use of the code for whatever purpose is
 * subject to separate license terms and conditions.
 *************************************************************************/
#include <stdio.h>
#include <string.h> 

#include "nrf_error.h"

#if defined(NUZZLE_COLLAR_PROTO_1) || defined(BOARD_PCA10028)

#include "spi_5W_master.h"

//#include "spi_master_if.h"

//#include "st_lis3dh.h"
#include "SensorUtils.h"
//#include "NZL_CollarAppEvents.h"

#if 0
#if defined(ACCELEROMETER_ENABLE)


/*
 *  Notes:
 *   1. The read/write buffer sizes need to be fine tuned later. TODO: 03/31/15 - MN
 * 
 *
 */
#define MAX_ACCEL_WRITE_BUFF_SIZE	(10) 
#define MAX_ACCEL_READ_BUFF_SIZE        (20) 

uint8_t		AccelWriteBuff[MAX_ACCEL_WRITE_BUFF_SIZE];
uint8_t     AccelReadBuff[MAX_ACCEL_READ_BUFF_SIZE];



// LIS3DH SPI flags for read/write and auto increment for multi-register reads
#define LIS3DH_SPI_READ_FLAG			(0x80)
#define LIS3DH_SPI_AUTO_INCREMENT_ADRR_FLAG	(0x40)


/*
 * Notes:
 * 1. Use a typical sized global write buffer to minimize malloc/free call overhead
 *    If required write buffer is longer than
 *
 * 2. Need a separate similar function for Accel data fetches from FIFO,
 *    AccelReadDataFifo().
 *    Here, register address will not be auto-incremented and also bit-alignment of 10-bit values
 *    may be useful to do here. Verify this and implement. TODO: 03/25/15 - MN 
 */ 
NZL_Return_t	AccelReadReg( uint8_t startReg, uint8_t numRegs, uint8_t *pReadBuff)
{
    NZL_Return_t    retStat = NZL_RET_SUCCESS;

//	uint8_t		txBuff[2];
	uint32_t	spiStat;

	// Setup Tx and Rx buffers
	// Setup read flag and auto-increment address, if reading multiple registers
    AccelWriteBuff[0] = startReg | LIS3DH_SPI_READ_FLAG | 
                        numRegs > 1 ? LIS3DH_SPI_AUTO_INCREMENT_ADRR_FLAG: 0x00;

//	txBuff[0] = startReg | LIS3DH_SPI_READ_FLAG | 
//	 			numRegs > 1 ? LIS3DH_SPI_AUTO_INCREMENT_ADRR_FLAG: 0x00;

	if ((spiStat = spi_master_send_recv(ACCEL_SPI_BUS, AccelWriteBuff, 1, pReadBuff, numRegs)) != NRF_SUCCESS)
	{
		D(printf("LIS3DH SPI ReadReg() error: %d!\n", spiStat));
        retStat = NZL_RET_ERROR;
	}

    return(retStat);

 
}
#endif  // defined(ACCELEROMETER_ENABLE)


#if 0
/*
 * SPI Slave Device Interface API called by device drivers specific to slave device
 */
/*
 * SPI device interface calls.
 * -- Move to AccelDrvr.c/.h
 */
#if defined(ACCELEROMETER_ENABLE)
//spi_master_config_t     accelSpiConfig = ACCEL_SPI_MASTER_CONFIG
void SPI_AccelIfInit( void )
{
    // spi_master_config_t     accelSpiConfig;
    spi_master_config_t     accelSpiConfig = ACCEL_SPI_MASTER_CONFIG;
    
#if 0    
    accelSpiConfig.SPI_Freq         = SPI_FREQUENCY_FREQUENCY_M1;
    accelSpiConfig.SPI_Pin_SCK      = ACCEL_SPI_SCK_PIN;
    accelSpiConfig.SPI_Pin_MISO     = ACCEL_SPI_MISO_PIN;
    accelSpiConfig.SPI_Pin_MOSI     = ACCEL_SPI_MOSI_PIN;
//    accelSpiConfig.SPI_Pin_SS       = ACCEL_SPI_SS_PIN;
    accelSpiConfig.SPI_Pin_SS       =  ACCEL_SPI_SS_PIN;                // 8u;
    accelSpiConfig.SPI_ORDER        = SPI_CONFIG_ORDER_MsbFirst;
    accelSpiConfig.SPI_CPOL         = SPI_CONFIG_CPOL_ActiveHigh;
    accelSpiConfig.SPI_CPHA         = SPI_CONFIG_CPHA_Leading;
#endif

    // Intialize & configure the SPI master module and bus interface signals, associated event handler
    SpiMasterInit(ACCEL_SPI_BUS, SpiMasterAccelEventHandler, &accelSpiConfig);


}

#endif  // defined(ACCELEROMETER_ENABLE)



/*
 * -- Move to AccelDrvr.c/.h
 * Check Accel sensor interface/availability
 * - Read WHO_AM_I reg and verify for LIS3DH.
 * if available/passed
 *    Perform Accel default initialization
 *   
 * else
 *   return NZL_DEV_ERR_ACCEL_NOT_AVAIL
 * endif
 * if success
 *
 *   Set Accel Sensor status flags
 * else
 *   Disable and power off SPI_AccelInterface and bus
 *   Set Accel Sensor status flags
 * endif

  - Do not check for bus availability, init status etc for now.
    These would be defined/setup in AccelSensorStatus struct TBD. TODO: 03/24/15 - MN


 */

/*
 *  Notes:
 *  1. The readBuff[] size needs to be at least one byte larger than the number of bytes expected
 *     to receive, since the first byte is always a 'dummy' byte to discard.
 *  2. Make the readBuff[] a local for now. Could use a global, if more efficient. TODO: 03/25/15 - MN
 *  3. When doing FIFO reads of large sample sets, do a malloc() for the required size for the 
 *     duration of the processing and free up at end.
 *  4. AccelSensorInit() will be called multiple times by MotionSense_SM()
 *     until AccelSensorInit() reports that ACCEL_SENSOR_INIT_SUCCESS.
 *  5. AccelSensorInit() will be called twice for accel sensor access SPI transfer
 *     
 *     
 */
NZL_Return_t    AccelSensorInit( void )
{
    NZL_Return_t    retStat = NZL_RET_SUCCESS;
    uint8_t         readBuff[2];
    

    memset(AccelReadBuff,0x00, sizeof(AccelReadBuff));
    // Read ID reg and verify
    AccelReadReg((uint8_t)LIS3DH_WHOAMI_REG, 1, AccelReadBuff);
    // Note: Since we are doing a non-blocking call completed through multiple interrupt handler calls
    //       we cannot check for read results yet. It needs to be done in the BSP events loop, when we
    //       get a SPI TRANSFER COMPLETED event is received.
    //       This needs to be tied to the sensor data read state logic. TODO. 03/31/15 - MN
//    if (readBuff[1] != LIS3DH_WHOAMI_VAL)
//        return(NZL_DEV_ERR_ACCEL_NOT_AVAIL);



    return(retStat);
}

#endif  // #if defined(NUZZLE_COLLAR_PROTO_1)  || defined(BOARD_PCA10028)

void AccelSensor_SM(void *pEventData, uint16_t eDatSize)
{

}
#endif
#endif