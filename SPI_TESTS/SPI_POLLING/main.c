/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @defgroup spi_master_example_main main.c
* @{
* @ingroup spi_master_example
*
* @brief SPI Master Loopback Example Application main file.
*
* This file contains the source code for a sample application using SPI.
*
*/

#include "nrf51.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "nordic_common.h"
//#include "nrf_sdm.h"
//#include "softdevice_handler.h"

//#include "SensorUtils.h"
#include <collar_defines.h>
#include <spi_master.h>


#define IS_SRVC_CHANGED_CHARACT_PRESENT     0     /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_TIMER_PRESCALER      0                      /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS     4      // BSP_APP_TIMERS_NUMBER  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE  2                      /**< Size of timer operation queues. */

#define DELAY_MS                 10          //1000                   /**< Timer Delay in milli-seconds. */


// LIS3DS0 register info
#define LIS3DH_WHOAMI_REG			(0x0F)		// [R/O -0x33] identification reg
#define LIS3DH_WHOAMI_VAL  			(0x33)		// identification value

// LIS3DH SPI flags for read/write and auto increment for multi-register reads
#define LIS3DH_SPI_READ_FLAG			    (0x80)
#define LIS3DH_SPI_AUTO_INCREMENT_ADRR_FLAG (0x40)



/** @def  TX_RX_MSG_LENGTH
 * number of bytes to transmit and receive. This amount of bytes will also be tested to see that
 * the received bytes from slave are the same as the transmitted bytes from the master */
#define TX_RX_MSG_LENGTH         100

static uint8_t SpiTxData[TX_RX_MSG_LENGTH]; /**< SPI master TX buffer. */
static uint8_t SpiRxData[TX_RX_MSG_LENGTH]; /**< SPI master RX buffer. */

#if 0
/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    UNUSED_VARIABLE(bsp_indication_set(BSP_INDICATE_FATAL_ERROR));

    for (;; )
    {
        // No implementation needed.
    }
}
#endif


/** @brief Function for main application entry.
 * Polling mode SPI master testing code.
 */
int main(void)
{
    NRF_SPI_Type *spiModAddr = NULL;  
    // Setup bsp module.
//    bsp_configuration();
    // Initialize the spi port to use for Accel
    // SPI_MASTER0, SPI_MODE0, LSB first
    spiModAddr = (NRF_SPI_Type *)spi_master_init(ACCEL_SPI_BUS, SPI_MODE0, false);
    for (;; )
    {
        // Copy register to read from (Accel sensor LSM3DH0)
        // Read one register: send address + read register value
        SpiTxData[0] = (uint8_t)(LIS3DH_WHOAMI_REG | LIS3DH_SPI_READ_FLAG);
        SpiTxData[1] = 0x00;
        SpiRxData[0] = 0x00;
        SpiRxData[1] = 0x00;

        spi_master_tx_rx((uint32_t *)spiModAddr, 2, SpiTxData, SpiRxData);

        nrf_delay_ms(DELAY_MS);
    }
}


/** @} */
