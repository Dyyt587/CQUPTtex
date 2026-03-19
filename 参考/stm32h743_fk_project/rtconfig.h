#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

#define SOC_STM32H743II

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_CPUS_NR 1
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 1024
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512

/* kservice optimization */

/* end of kservice optimization */

/* klibc optimization */

#define RT_KLIBC_USING_STDLIB
#define RT_KLIBC_USING_PRINTF_LONGLONG
/* end of klibc optimization */
#define RT_USING_DEBUG
#define RT_DEBUGING_ASSERT
#define RT_DEBUGING_COLOR
#define RT_DEBUGING_CONTEXT

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
/* end of Inter-Thread communication */

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP
#define RT_USING_MEMTRACE
#define RT_USING_HEAP_ISR
#define RT_USING_HEAP
/* end of Memory Management */
#define RT_USING_DEVICE
#define RT_USING_INTERRUPT_INFO
#define RT_USING_THREADSAFE_PRINTF
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x50200
#define RT_BACKTRACE_LEVEL_MAX_NR 32
/* end of RT-Thread Kernel */
#define RT_USING_CACHE
#define RT_USING_CPU_FFS
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 80
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10
#define FINSH_USING_OPTION_COMPLETION

/* DFS: device virtual file system */

#define RT_USING_DFS
#define DFS_USING_POSIX
#define DFS_USING_WORKDIR
#define DFS_FD_MAX 16
#define RT_USING_DFS_V1
#define DFS_FILESYSTEMS_MAX 4
#define DFS_FILESYSTEM_TYPES_MAX 4
#define RT_USING_DFS_DEVFS
/* end of DFS: device virtual file system */
#define RT_USING_FAL
#define FAL_DEBUG_CONFIG
#define FAL_DEBUG 1
#define FAL_PART_HAS_TABLE_CFG
#define FAL_USING_SFUD_PORT
#define FAL_USING_NOR_FLASH_DEV_NAME "W25Q256"

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_UNAMED_PIPE_NUMBER 64
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V2
#define RT_SERIAL_USING_DMA
#define RT_USING_I2C
#define RT_USING_ADC
#define RT_USING_PWM
#define RT_USING_RTC
#define RT_USING_SPI
#define RT_USING_QSPI
#define RT_USING_SFUD
#define RT_SFUD_USING_SFDP
#define RT_SFUD_USING_FLASH_INFO_TABLE
#define RT_SFUD_USING_QSPI
#define RT_SFUD_SPI_MAX_HZ 50000000
#define RT_USING_LCD
#define RT_USING_PIN
#define RT_USING_KTIME

/* Using USB */

#define RT_USING_USB
#define RT_USING_USB_DEVICE
#define RT_USBD_THREAD_STACK_SZ 4096
#define USB_VENDOR_ID 0x0FFE
#define USB_PRODUCT_ID 0x0001
#define _RT_USB_DEVICE_NONE
#define RT_USB_DEVICE_NONE
/* end of Using USB */
/* end of Device Drivers */

/* C/C++ and POSIX layer */

/* ISO-ANSI C layer */

/* Timezone and Daylight Saving Time */

#define RT_LIBC_USING_LIGHT_TZ_DST
#define RT_LIBC_TZ_DEFAULT_HOUR 8
#define RT_LIBC_TZ_DEFAULT_MIN 0
#define RT_LIBC_TZ_DEFAULT_SEC 0
/* end of Timezone and Daylight Saving Time */
/* end of ISO-ANSI C layer */

/* POSIX (Portable Operating System Interface) layer */

#define RT_USING_POSIX_FS
#define RT_USING_POSIX_DEVIO
#define RT_USING_POSIX_STDIO
#define RT_USING_POSIX_DELAY
#define RT_USING_POSIX_CLOCK
#define RT_USING_PTHREADS
#define PTHREAD_NUM_MAX 8

/* Interprocess Communication (IPC) */


/* Socket is in the 'Network' category */

/* end of Interprocess Communication (IPC) */
/* end of POSIX (Portable Operating System Interface) layer */
#define RT_USING_CPLUSPLUS
#define RT_USING_CPLUSPLUS11
#define RT_USING_CPP_WRAPPER
#define RT_USING_CPP_EXCEPTIONS
/* end of C/C++ and POSIX layer */

/* Network */

/* end of Network */

/* Memory protection */

/* end of Memory protection */

/* Utilities */

#define RT_USING_ULOG
#define ULOG_OUTPUT_LVL_D
#define ULOG_OUTPUT_LVL 7
#define ULOG_USING_ISR_LOG
#define ULOG_ASSERT_ENABLE
#define ULOG_LINE_BUF_SIZE 128
#define ULOG_USING_ASYNC_OUTPUT
#define ULOG_ASYNC_OUTPUT_BUF_SIZE 4096
#define ULOG_ASYNC_OUTPUT_BY_THREAD
#define ULOG_ASYNC_OUTPUT_THREAD_STACK 1024
#define ULOG_ASYNC_OUTPUT_THREAD_PRIORITY 30

/* log format */

#define ULOG_OUTPUT_FLOAT
#define ULOG_USING_COLOR
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG
#define ULOG_OUTPUT_THREAD_NAME
/* end of log format */
#define ULOG_BACKEND_USING_CONSOLE
#define ULOG_USING_FILTER
#define RT_USING_ADT
#define RT_USING_ADT_AVL
#define RT_USING_ADT_BITMAP
#define RT_USING_ADT_HASHMAP
#define RT_USING_ADT_REF
/* end of Utilities */
/* end of RT-Thread Components */

/* RT-Thread online packages */

/* IoT - internet of things */

#define PKG_USING_FREEMODBUS
#define PKG_MODBUS_MASTER

/* advanced configuration */

#define RT_M_DISCRETE_INPUT_START 0
#define RT_M_DISCRETE_INPUT_NDISCRETES 8
#define RT_M_COIL_START 0
#define RT_M_COIL_NCOILS 64
#define RT_M_REG_INPUT_START 0
#define RT_M_REG_INPUT_NREGS 100
#define RT_M_REG_HOLDING_START 0
#define RT_M_REG_HOLDING_NREGS 406
#define RT_M_HD_RESERVE 0
#define RT_M_IN_RESERVE 0
#define RT_M_CO_RESERVE 0
#define RT_M_DI_RESERVE 0
/* end of advanced configuration */
#define PKG_MODBUS_MASTER_RTU
#define RT_MODBUS_MASTER_USE_CONTROL_PIN
#define MODBUS_MASTER_RT_CONTROL_PIN_INDEX 52
#define PKG_MODBUS_MASTER_SAMPLE
#define MB_SAMPLE_TEST_SLAVE_ADDR 1
#define MB_MASTER_USING_PORT_NUM 2
#define MB_MASTER_USING_PORT_BAUDRATE 115200
#define PKG_USING_FREEMODBUS_LATEST_VERSION

/* Wi-Fi */

/* Marvell WiFi */

/* end of Marvell WiFi */

/* Wiced WiFi */

/* end of Wiced WiFi */

/* CYW43012 WiFi */

/* end of CYW43012 WiFi */

/* BL808 WiFi */

/* end of BL808 WiFi */

/* CYW43439 WiFi */

/* end of CYW43439 WiFi */
/* end of Wi-Fi */

/* IoT Cloud */

/* end of IoT Cloud */
/* end of IoT - internet of things */

/* security packages */

/* end of security packages */

/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* end of JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* XML: Extensible Markup Language */

/* end of XML: Extensible Markup Language */
/* end of language packages */

/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */

/* end of LVGL: powerful and easy-to-use embedded GUI library */

/* u8g2: a monochrome graphic library */

/* end of u8g2: a monochrome graphic library */
/* end of multimedia packages */

/* tools packages */

#define PKG_USING_CPU_USAGE
#define PKG_USING_CPU_USAGE_LATEST_VERSION
/* end of tools packages */

/* system packages */

/* enhanced kernel services */

#define PKG_USING_RT_MEMCPY_CM
#define PKG_USING_RT_MEMCPY_CM_LATEST_VERSION
#define PKG_USING_RT_KPRINTF_THREADSAFE
#define PKG_USING_RT_KPRINTF_THREADSAFE_LATEST_VERSION
#define PKG_USING_RT_VSNPRINTF_FULL
#define PKG_VSNPRINTF_SUPPORT_DECIMAL_SPECIFIERS
#define PKG_VSNPRINTF_SUPPORT_EXPONENTIAL_SPECIFIERS
#define PKG_VSNPRINTF_SUPPORT_WRITEBACK_SPECIFIER
#define PKG_VSNPRINTF_SUPPORT_LONG_LONG
#define PKG_VSNPRINTF_CHECK_FOR_NUL_IN_FORMAT_SPECIFIER
#define PKG_VSNPRINTF_INTEGER_BUFFER_SIZE 32
#define PKG_VSNPRINTF_DECIMAL_BUFFER_SIZE 32
#define PKG_VSNPRINTF_DEFAULT_FLOAT_PRECISION 6
#define PKG_VSNPRINTF_MAX_INTEGRAL_DIGITS_FOR_DECIMAL 9
#define PKG_VSNPRINTF_LOG10_TAYLOR_TERMS 4
#define PKG_USING_RT_VSNPRINTF_FULL_LATEST_VERSION
/* end of enhanced kernel services */

/* acceleration: Assembly language or algorithmic acceleration packages */

/* end of acceleration: Assembly language or algorithmic acceleration packages */

/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

/* end of CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

/* Micrium: Micrium software products porting for RT-Thread */

/* end of Micrium: Micrium software products porting for RT-Thread */
#define PKG_USING_PERF_COUNTER
#define PKG_PERF_COUNTER_USING_THREAD_STATISTIC
#define PKG_USING_PERF_COUNTER_V2241
#define PKG_USING_FLASHDB
#define FDB_USING_KVDB
#define FDB_KV_AUTO_UPDATE
#define FDB_USING_TSDB
#define FDB_USING_FAL_MODE
#define FDB_WRITE_GRAN_8BITS
#define FDB_WRITE_GRAN 8
#define FDB_NOT_USING_FILE_MODE
#define FLASHDB_USING_SAMPLES
#define FDB_DEBUG_ENABLE
#define PKG_USING_FLASHDB_V10102
#define PKG_FLASHDB_VER_NUM 0x10102
/* end of system packages */

/* peripheral libraries and drivers */

/* HAL & SDK Drivers */

/* STM32 HAL & SDK Drivers */

/* end of STM32 HAL & SDK Drivers */

/* Infineon HAL Packages */

/* end of Infineon HAL Packages */

/* Kendryte SDK */

/* end of Kendryte SDK */
/* end of HAL & SDK Drivers */

/* sensors drivers */

/* end of sensors drivers */

/* touch drivers */

/* end of touch drivers */
#define PKG_USING_PCF8574
#define PKG_USING_PCF8574_LATEST_VERSION
#define PKG_USING_RS485
#define RS485_USING_TEST
#define RS485_TEST_SERIAL "uart2"
#define RS485_TEST_BAUDRATE 115200
#define RS485_TEST_PARITY 0
#define RS485_TEST_PIN 52
#define RS485_TEST_LEVEL 1
#define RS485_USING_SAMPLE_MASTER
#define RS485_SAMPLE_MASTER_SERIAL "uart2"
#define RS485_SAMPLE_MASTER_BAUDRATE 115200
#define RS485_SAMPLE_MASTER_PARITY 0
#define RS485_SAMPLE_MASTER_PIN 52
#define RS485_SAMPLE_MASTER_LVL 1
#define PKG_USING_RS485_LATEST_VERSION
#define PKG_USING_ST7789
#define PKG_ST_7789_SPI_BUS_NAME "spi5"
#define PKG_ST_7789_SPI_DEVICE_NAME "spi50"
#define PKG_ST_7789_WIDTH 240
#define PKG_ST_7789_HEIGHT 320
#define PKG_ST_7789_DC_PIN -1
#define PKG_ST_7789_RES_PIN -1
#define PKG_ST_7789_CS_PIN -1
#define PKG_ST_7789_BLK_PIN -1
/* end of peripheral libraries and drivers */

/* AI packages */

/* end of AI packages */

/* Signal Processing and Control Algorithm Packages */

#define PKG_USING_APID
#define PKG_USING_APID_LATEST_VERSION
/* end of Signal Processing and Control Algorithm Packages */

/* miscellaneous packages */

/* project laboratory */

/* end of project laboratory */

/* samples: kernel and components samples */

/* end of samples: kernel and components samples */

/* entertainment: terminal games and other interesting software packages */

/* end of entertainment: terminal games and other interesting software packages */
#define PKG_USING_FLEXIBLE_BUTTON
#define PKG_USING_FLEXIBLE_BUTTON_LATEST
#define PKG_USING_GET_IRQ_PRIORITY
#define PKG_USING_GET_IRQ_PRIORITY_LATEST_VERSION
/* end of miscellaneous packages */

/* Arduino libraries */


/* Projects and Demos */

/* end of Projects and Demos */

/* Sensors */

/* end of Sensors */

/* Display */

/* end of Display */

/* Timing */

/* end of Timing */

/* Data Processing */

/* end of Data Processing */

/* Data Storage */

/* Communication */

/* end of Communication */

/* Device Control */

/* end of Device Control */

/* Other */

/* end of Other */

/* Signal IO */

/* end of Signal IO */

/* Uncategorized */

/* end of Arduino libraries */
/* end of RT-Thread online packages */
#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32H7

/* Hardware Drivers Config */

/* Onboard Peripheral Drivers */

#define BSP_USING_COM2
#define BSP_USING_QSPI_FLASH
/* end of Onboard Peripheral Drivers */

/* On-chip Peripheral Drivers */

#define BSP_USING_GPIO
#define BSP_USING_PWM
#define BSP_USING_PWM2
#define BSP_USING_PWM2_CH1
#define BSP_USING_PWM2_CH3
#define BSP_USING_PWM2_CH4
#define BSP_USING_PWM3
#define BSP_USING_PWM3_CH1
#define BSP_USING_PWM3_CH2
#define BSP_USING_PWM3_CH3
#define BSP_USING_PWM3_CH4
#define BSP_USING_PWM5
#define BSP_USING_PWM5_CH1
#define BSP_USING_PWM5_CH2
#define BSP_USING_PWM5_CH3
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_UART1_RX_USING_DMA
#define BSP_UART1_TX_USING_DMA
#define BSP_UART1_RX_BUFSIZE 4096
#define BSP_UART1_TX_BUFSIZE 4096
#define BSP_USING_UART2
#define BSP_UART2_TX_USING_DMA
#define BSP_UART2_RX_BUFSIZE 4096
#define BSP_UART2_TX_BUFSIZE 0
#define BSP_USING_UART3
#define BSP_UART3_RX_BUFSIZE 4096
#define BSP_UART3_TX_BUFSIZE 0
#define BSP_USING_UART4
#define BSP_UART4_RX_BUFSIZE 256
#define BSP_UART4_TX_BUFSIZE 256
#define BSP_USING_UART5
#define BSP_UART5_RX_USING_DMA
#define BSP_UART5_RX_BUFSIZE 1024
#define BSP_UART5_TX_BUFSIZE 0
#define BSP_USING_UART6
#define BSP_UART6_RX_BUFSIZE 1024
#define BSP_UART6_TX_BUFSIZE 0
#define BSP_USING_UART7
#define BSP_UART7_RX_USING_DMA
#define BSP_UART7_RX_BUFSIZE 1024
#define BSP_UART7_TX_BUFSIZE 0
#define BSP_USING_SPI
#define BSP_USING_SPI5
#define BSP_SPI5_TX_USING_DMA
#define BSP_USING_QSPI
#define BSP_USING_I2C1
#define BSP_USING_ON_CHIP_FLASH
/* end of On-chip Peripheral Drivers */

/* Board extended module Drivers */

/* end of Hardware Drivers Config */

#endif
