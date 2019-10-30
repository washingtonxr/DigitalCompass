#ifndef ak09918_H
#define ak09918_H
#if 0
#include "stdio.h"
#else
#include "project.h"
#include "KX022.h"
#endif
#include "CAlgorithmCom.h"

/* AK09918 I2C Register List. */
#define AK09918_WIA1        0x00    // Company ID
#define AK09918_WIA2        0x01    // Device ID
#define AK09918_RSV1        0x02    // Reserved 1
#define AK09918_RSV2        0x03    // Reserved 2
#define AK09918_ST1         0x10    // DataStatus 1
#define AK09918_HXL         0x11    // X-axis data 
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13    // Y-axis data
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15    // Z-axis data
#define AK09918_HZH         0x16
#define AK09918_TMPS        0x17    // Dummy
#define AK09918_ST2         0x18    // Datastatus 2
#define AK09918_CNTL1       0x30    // Dummy
#define AK09918_CNTL2       0x31    // Control settings
#define AK09918_CNTL3       0x32    // Control settings

#define AK09918_SRST_BIT    0x01    // Soft Reset
#define AK09918_HOFL_BIT    0x08    // Sensor Over Flow
#define AK09918_DOR_BIT     0x02    // Data Over Run
#define AK09918_DRDY_BIT    0x01    // Data Ready

/* AK09918 I2C address. */
#define AK09918_I2C_ADDR	0x0C	// I2C address (Can't be changed)

// AK09918 has following seven operation modes:
// (1) Power-down mode: AK09918 doesn't measure
// (2) Single measurement mode: measure when you call any getData() function
// (3) Continuous measurement mode 1: 10Hz,  measure 10 times per second,
// (4) Continuous measurement mode 2: 20Hz,  measure 20 times per second,
// (5) Continuous measurement mode 3: 50Hz,  measure 50 times per second,
// (6) Continuous measurement mode 4: 100Hz, measure 100 times per second,
// (7) Self-test mode
typedef enum {
	AK09918_POWER_DOWN       = 0x00,
	AK09918_NORMAL           = 0x01,
	AK09918_CONTINUOUS_10HZ  = 0x02,
	AK09918_CONTINUOUS_20HZ  = 0x04,
	AK09918_CONTINUOUS_50HZ  = 0x06,
	AK09918_CONTINUOUS_100HZ = 0x08,
	AK09918_SELF_TEST        = 0x10,
} AK09918_mode_type_t;

typedef enum {
	AK09918_ERR_OK              = 0, // OK
	AK09918_ERR_DOR,                 // data skipped
	AK09918_ERR_NOT_RDY,             // not ready
	AK09918_ERR_TIMEOUT,             // read/write timeout
	AK09918_ERR_SELFTEST_FAILED,     // self test failed
	AK09918_ERR_OVERFLOW,            // sensor overflow, means |x|+|y|+|z| >= 4912uT
	AK09918_ERR_WRITE_FAILED,        // fail to write
	AK09918_ERR_READ_FAILED,         // fail to read
	AK09918_ERR_UNKNOWN,             // unknown error
} AK09918_err_type_t;
    
typedef struct{
    double axis[3];
}AK09918_axis_t;

typedef struct {
    uint8_t addr;
    AK09918_mode_type_t mode;
#if 1
    AK09918_axis_t value;
#endif
} AK09918_dev_t;

int ak09918_init(AK09918_dev_t *dev);
int ak09918_power_down(AK09918_dev_t *dev);

int ak09918_get_data(AK09918_dev_t *dev);
uint16_t ak09918_get_direction(axis_dblock_t *GDB, AK09918_dev_t *CDB);

#endif
/* End of this file. */
