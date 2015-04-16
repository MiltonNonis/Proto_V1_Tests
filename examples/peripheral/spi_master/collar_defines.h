/* Pinout definition file for PetPOMM collar v1.0 */
#ifndef __COLLAR_PINOUT_H

#define __COLLAR_PINOUT_H

/* Test environment settings */
#define DEBUG // Enable to show print statements. Must be off for real builds
//#define BT_ENABLE

//#define ACCELEROMETER_ENABLE
//#define TEMP_ENABLE
//#define GPS_ENABLE
//#define CELL_ENABLE

// Don't touch
#ifdef DEBUG
# define D(x) x
#else
# define D(x)
#endif

/* Timer Params */
#define BATTERY_TIMER                   120
#define GPS_TIMER                       120
#define ACCELEROMETER_TIMER             2.6
#define ACCELEROMETER_PROCESS_TIMER     60
#define TEMP_TIMER                      60

#define DATA_TRANSMISSION_TIMER         300

//#define GPS_FIX_INTERVAL                "0000012C" // 300s - Auto get a new fix at this interval (s)
#define GPS_FIX_INTERVAL                "00000078" // 120s - Auto get a new fix at this interval (s)

#define GPS_MAX_SEARCH_TIME             "0000AFC8" // 45s - Search for signal for this length (*ms*)
#define GPS_FAIL_TIME_OFF               "000493E0" // 300s - When GPS fails, don't try again for this long (*ms*)


/* End Test environment settings */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_MAX_TIMERS                 7                                         /**< Maximum number of simultaneously created timers. */
//#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

/******* BEGIN PINOUT *********/

#define HORNET_GPS_UART_TX_PIN  16
#define HORNET_GPS_UART_RX_PIN  18

#define QUECTEL_M66_UART_TX_PIN 22
#define QUECTEL_M66_UART_RX_PIN 23
#define QUECTEL_M66_DTR         20 // Used to signal Low Power mode. Active High

/* I2C used for the LIS3DH accelerometer and TMP103 temp sensor */

#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER      29
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER       28

#define LIS3DH_I2C_ADDR 0x32    // Base address x19
#define TMP103_I2C_ADDR 0xE0    // Base address x30

/* FIFO Params */
#define GPS_BUFFER_DEPTH 25  // 1x every 5 minutes, ~ 4 hours worth      
#define TEMP_DATA_BUFFER_DEPTH 256 // 1 activity levels per byte, ~ 4 hours worth @ 1 minute intervals
#define ACCEL_DATA_BUFFER_DEPTH 128 // 2 activity levels per byte, ~4 hours worth @ 1 minute intervals
#define ACCEL_DATE_BUFFER_DEPTH 256 // 1 byte per date, 1 date per activity update

//typedef char gps_lat_buffer[GPS_BUFFER_DEPTH];
//typedef char gps_lng_buffer[GPS_BUFFER_DEPTH];

/* SPI bus GPIO pin defines for LIS3DH accelerometer */
#if defined(NUZZLE_COLLAR_PROTO_1)
#define SPIM0_SCK_PIN                           (5u)     //[P0.05/AIN6      : Pin # 9] SPI clock GPIO pin. 
#define SPIM0_MOSI_PIN                          (6u)     //[P0.06/AIN7/AREF1: Pin #10] SPI Master Out Slave In GPIO pin. 
#define SPIM0_MISO_PIN                          (7u)     //[P0.07           : Pin #11] SPI Master In Slave Out GPIO pin.
												
#if defined(ACCELEROMETER_ENABLE)
#define SPIM0_SS_ACCEL_PIN                      (8u)     //[P0.08           : Pin #14] SPI Slave Select ACCEL GPIO pin. 
#endif  // #if defined(ACCELEROMETER_ENABLE)
#if defined(GPS_ENABLE)
#define SPIM0_SS_GPS_PIN                        (13u)    //[P0.13           : Pin #19] SPI Slave Select GPS GPIO pin. 
#endif

#endif	// #if defined(NUZZLE_COLLAR_PROTO_1)

#if defined(BOARD_PCA10028)
#define SPIM0_SCK_PIN                           (1u)     //[P0.01/AIN2      : Pin # 5] SPI clock GPIO pin. 
#define SPIM0_MOSI_PIN                          (2u)     //[P0.02/AIN3      : Pin # 6] SPI Master Out Slave In GPIO pin. 
#define SPIM0_MISO_PIN                          (3u)     //[P0.03/AIN4      : Pin # 7] SPI Master In Slave Out GPIO pin.
												
#if defined(ACCELEROMETER_ENABLE)
#define SPIM0_SS_ACCEL_PIN                      (4u)     //[P0.04/AIN5      : Pin # 8] SPI Slave Select ACCEL GPIO pin. 
#endif  // #if defined(ACCELEROMETER_ENABLE)

#endif	// #if defined(BOARD_PCA10028)


#if defined(ACCELEROMETER_ENABLE)
#define ACCEL_SPI_SCK_PIN                      	(SPIM0_SCK_PIN)     
#define ACCEL_SPI_MOSI_PIN                     	(SPIM0_MOSI_PIN)    
#define ACCEL_SPI_MISO_PIN                     	(SPIM0_MISO_PIN)
#define ACCEL_SPI_SS_PIN                     	(SPIM0_SS_ACCEL_PIN)
#endif




/* Test board pins */
#define WAKEUP_BUTTON_PIN                       0          /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID               1          /**< Button used for deleting all bonded centrals during startup. */

#define ADVERTISING_LED_PIN_NO                  8          /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO                    9          /**< Is on when device has connected. */
#define ASSERT_LED_PIN_NO                       15         /**< Is on when application has asserted. */

#define ACCEL_ACTIVITY_LED                      8
#define TEMP_ACTIVITY_LED                       9
#define GPS_ACTIVITY_LED                        10
#define CELL_ACTIVITY_LED                       11
// #define ACTIVITY_LEDS[4] =                    {12,13,14,15}; // in the st_lis3dh.h file

#define SER_CONN_ASSERT_LED_PIN                 11         /** Assertion error LED */
#define BUTTON_PULL                             NRF_GPIO_PIN_NOPULL
/* End test board pins */

/******** END PINOUT ********/

/**** Shared buffer / variables ****/
//#define UART_BUFFER 1200
//extern char uart_data;

#endif