/*****************************************************************************
 *  File name : SensorUtils.h
 *  Desc      : Nuzzle Collar Sensor Driver support utility functions.
 *              This driver code implementation is generic for all sensors on Collar.
 *  Dates : 03/22/15 - 03/25/15
 *        - Milton Nonis
 * Copyright (c) Nuzzle, Inc. 2015
 * All rights reserved. This code is Livescribe, Inc proprietary and confidential, 
 * and may be protected by patents. Any use of the code for whatever purpose is
 * subject to separate license terms and conditions.
 *************************************************************************/
#ifndef SENSOR_UTILS_H__
#define SENSOR_UTILS_H__ 
#include <stdio.h>
#include <string.h> 

#include "nrf_error.h"
#include "nrf51.h" 
#include "nrf51_bitfields.h" 
#include "collar_defines.h" 

/*
 * -- Move to .....
 * Nuzzle Collar API Return Defines
 * Notes:
 *  - Use defines for now. short int values.
 *  - May change to an enum later, if more suitable.
 */ 
#define NZL_RET_SUCCESS                         (0x0000) 
#define NZL_RET_ERROR                           (0x1000)  
#define NZL_DEV_ERR_ACCEL_NOT_AVAIL             (0x1001)

typedef uint16_t    NZL_Return_t;

#if defined(NUZZLE_COLLAR_PROTO_1) || defined(BOARD_PCA10028)

void            SPI_AccelIfInit( void );
NZL_Return_t	AccelReadReg( uint8_t startReg, uint8_t numRegs, uint8_t *pReadBuff);




#if 1
#if defined(ACCELEROMETER_ENABLE)

#define ACCEL_SPI_MASTER_CONFIG                                         \
{                                                                       \
    SPI_FREQUENCY_FREQUENCY_M1, /**< Serial clock frequency 1 Mbps. */  \
    ACCEL_SPI_SCK_PIN,          /**< SCK pin.  */                       \
    ACCEL_SPI_MISO_PIN,         /**< MISO pin  */                       \
    ACCEL_SPI_MOSI_PIN,         /**< MOSI pin  */                       \
    ACCEL_SPI_SS_PIN,           /**< Slave select pin */                \
    SPI_CONFIG_ORDER_MsbFirst,  /**< Bits order MSB. */                 \
    SPI_CONFIG_CPOL_ActiveHigh, /**< Serial clock polarity ACTIVEHIGH*/ \
    SPI_CONFIG_CPHA_Leading    /**< Serial clock phase LEADING. */      \
};
#endif

#if defined(GPS_ENABLE)
#define GPS_SPI_MASTER_CONFIG
{                                                                               \
    SPI_FREQUENCY_FREQUENCY_M1, /**< Serial clock frequency 1 Mbps. */          \
    GPS_SPI_SCK_PIN,          	/**< SCK pin. @See collar_defines.h */          \
    GPS_SPI_MISO_PIN,         	/**< MISO pin @See collar_defines.h */          \
    GPS_SPI_MOSI_PIN,         	/**< MOSI pin @See collar_defines.h */          \
    GPS_SPI_SS_PIN,           	/**< Slave select pin @See collar_defines.h */  \
    SPI_CONFIG_ORDER_MsbFirst,  /**< Bits order MSB. Note: This needs to be verified. TODO: 03/25/15 - MN*/                         \
    SPI_CONFIG_CPOL_ActiveHigh, /**< Serial clock polarity ACTIVEHIGH. */       \
    SPI_CONFIG_CPHA_Leading     /**< Serial clock phase LEADING. */             \
};
#endif
#endif


//spi_master_hw_instance_t    ACCEL_SPI_BUS = SPI_MASTER_0;
#define ACCEL_SPI_BUS       SPI_MASTER_0

#endif // #if defined(NUZZLE_COLLAR_PROTO_1) || defined(BOARD_PCA10028)
#endif // #define SENSOR_UTILS_H__